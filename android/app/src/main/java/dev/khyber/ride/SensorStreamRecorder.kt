package dev.khyber.ride

import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener2
import android.hardware.SensorManager
import android.os.Handler
import android.os.HandlerThread
import android.os.SystemClock
import android.util.Log
import dev.khyber.ride.proto.SensorInfo
import dev.khyber.ride.proto.StreamInfo
import java.io.File
import java.util.concurrent.CountDownLatch
import java.util.concurrent.TimeUnit
import uniffi.ride_ffi.SensorChunkBuilder
import uniffi.ride_ffi.StreamWriter

/**
 * Records one sensor to its own stream file, on its own thread, writing a chunk every [CHUNK_NS] of samples
 * or every [CHUNK_NS] of wall time for sensors that only report on change.
 *
 * Batching (`maxReportLatencyUs`) is lossless as long as the hardware FIFO doesn't overflow,
 * so latency is sized well under the FIFO, and halved whenever a gap shows up in the timestamps.
 */
class SensorStreamRecorder(
    private val sensorManager: SensorManager,
    private val sensor: Sensor,
    val name: String,
    dir: File,
    /** How many sensors share the FIFO at full rate, to size the batching latency. */
    sharingFifo: Int,
) : SensorEventListener2 {
    private val thread = HandlerThread("sensor-$name").apply { start() }
    private val handler = Handler(thread.looper)
    // The format is written by Rust (`crates/session/src/write.rs`), the same code the offline tools read with.
    private val writer = StreamWriter.create(dir.path, name)
    private val builder = SensorChunkBuilder()
    private var chunkStartNs = 0L
    private var lastTimestampNs = 0L
    private val continuous = sensor.reportingMode == Sensor.REPORTING_MODE_CONTINUOUS
    private var maxReportLatencyUs = initialLatencyUs(sensor, sharingFifo)
    private var reregisterAfterFlush = false

    @Volatile
    private var stopping: CountDownLatch? = null

    @Volatile
    var sampleCount = 0L
        private set

    @Volatile
    var gapCount = 0L
        private set

    val streamInfo: StreamInfo
        get() = StreamInfo.newBuilder().setName(name).setSensor(sensorInfo()).build()

    private val flushTask = object : Runnable {
        override fun run() {
            Health.guard("write $name") { flush() }
            handler.postDelayed(this, CHUNK_NS / 1_000_000)
        }
    }

    fun start() {
        Health.guard("sensor $name: register") { register() }
        handler.postDelayed(flushTask, CHUNK_NS / 1_000_000)
    }

    /**
     * Starts draining the hardware FIFO, since unregistering drops batched events still in it.
     * Call on every recorder before [stop], so the flushes run in parallel.
     */
    fun beginStop() {
        val latch = CountDownLatch(1)
        stopping = latch
        handler.post {
            if (Health.guard("sensor $name: flush FIFO") { sensorManager.flush(this) } != true) latch.countDown()
        }
    }

    fun stop() {
        if (stopping == null) beginStop()
        if (stopping?.await(STOP_FLUSH_TIMEOUT_S, TimeUnit.SECONDS) != true) {
            Log.w("SensorStreamRecorder", "$name: FIFO flush didn't complete before stop")
        }
        sensorManager.unregisterListener(this)
        handler.removeCallbacks(flushTask)
        handler.post {
            Health.guard("write $name") { flush() }
            Health.guard("close $name") { writer.finish() }
            writer.close()
            builder.close()
        }
        thread.quitSafely()
        thread.join()
    }

    private fun register() {
        sensorManager.registerListener(this, sensor, SAMPLING_PERIOD_US, maxReportLatencyUs, handler)
    }

    override fun onSensorChanged(event: SensorEvent) {
        Health.guard("sensor $name") { onSample(event) }
    }

    private fun onSample(event: SensorEvent) {
        if (!builder.fits(event.values.size.toUInt())) flush()
        if (builder.sampleCount() == 0u) chunkStartNs = event.timestamp
        if (continuous && lastTimestampNs != 0L && sensor.minDelay > 0) {
            val dtNs = event.timestamp - lastTimestampNs
            if (dtNs > GAP_FACTOR * sensor.minDelay * 1_000L) onGap(dtNs)
        }
        lastTimestampNs = event.timestamp
        // Raw bits (`floatToRawIntBits` keeps NaN payloads), so every value reaches the file exactly.
        builder.add(event.timestamp, event.values.map { it.toRawBits().toUInt() }, event.accuracy)
        sampleCount++
        LiveSpeed.onSensorEvent(event)
        if (event.timestamp - chunkStartNs >= CHUNK_NS) flush()
    }

    private fun onGap(dtNs: Long) {
        gapCount++
        if (maxReportLatencyUs == 0) return
        maxReportLatencyUs /= 2
        if (maxReportLatencyUs < MIN_LATENCY_US) maxReportLatencyUs = 0
        Log.w("SensorStreamRecorder", "$name: ${dtNs / 1_000_000} ms gap, batching latency now $maxReportLatencyUs us")
        // Changing latency needs a re-registration, which could drop events still in the FIFO,
        // so drain the FIFO first and re-register once the flush completes.
        if (!reregisterAfterFlush) {
            reregisterAfterFlush = true
            sensorManager.flush(this)
        }
    }

    override fun onFlushCompleted(sensor: Sensor) {
        Health.guard("sensor $name: flush completed") { onFlushed() }
    }

    private fun onFlushed() {
        stopping?.let {
            it.countDown()
            return
        }
        if (!reregisterAfterFlush) return
        reregisterAfterFlush = false
        sensorManager.unregisterListener(this)
        // The re-registration itself leaves a short gap; don't count it as another overflow.
        lastTimestampNs = 0L
        register()
    }

    override fun onAccuracyChanged(sensor: Sensor, accuracy: Int) {}

    private fun flush() {
        writer.appendSensorChunk(builder, SystemClock.elapsedRealtimeNanos())
    }

    private fun sensorInfo(): SensorInfo = SensorInfo.newBuilder()
        .setType(sensor.type)
        .setStringType(sensor.stringType)
        .setName(sensor.name)
        .setVendor(sensor.vendor)
        .setVersion(sensor.version)
        .setResolution(sensor.resolution)
        .setMaximumRange(sensor.maximumRange)
        .setPowerMa(sensor.power)
        .setMinDelayUs(sensor.minDelay)
        .setMaxDelayUs(sensor.maxDelay)
        .setFifoMaxEventCount(sensor.fifoMaxEventCount)
        .setFifoReservedEventCount(sensor.fifoReservedEventCount)
        .setReportingMode(sensor.reportingMode)
        .setWakeUp(sensor.isWakeUpSensor)
        .setSamplingPeriodUs(SAMPLING_PERIOD_US)
        .setMaxReportLatencyUs(maxReportLatencyUs)
        .build()

    companion object {
        /** Fastest rate the sensor supports. */
        const val SAMPLING_PERIOD_US = 0
        const val CHUNK_NS = 2_000_000_000L
        private const val MAX_LATENCY_US = 1_000_000
        private const val MIN_LATENCY_US = 20_000
        private const val GAP_FACTOR = 4
        private const val STOP_FLUSH_TIMEOUT_S = 3L

        /** A quarter of the time the FIFO takes to fill at full rate, capped at [MAX_LATENCY_US]; 0 without a FIFO. */
        fun initialLatencyUs(sensor: Sensor, sharingFifo: Int): Int {
            if (sensor.minDelay <= 0) return 0
            val reserved = sensor.fifoReservedEventCount
            val events = if (reserved > 0) reserved else sensor.fifoMaxEventCount / maxOf(sharingFifo, 1)
            if (events <= 0) return 0
            val fillUs = events.toLong() * sensor.minDelay
            return minOf(fillUs / 4, MAX_LATENCY_US.toLong()).toInt().let { if (it < MIN_LATENCY_US) 0 else it }
        }
    }
}
