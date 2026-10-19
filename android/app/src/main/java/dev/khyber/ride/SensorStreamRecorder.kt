package dev.khyber.ride

import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import android.os.Handler
import android.os.HandlerThread
import android.os.SystemClock
import dev.khyber.ride.proto.SensorInfo
import dev.khyber.ride.proto.StreamInfo
import java.io.File

/** Records one sensor to its own stream file, on its own thread, writing a chunk every [CHUNK_NS]. */
class SensorStreamRecorder(
    private val sensorManager: SensorManager,
    private val sensor: Sensor,
    private val name: String,
    private val valuesPerSample: Int,
    dir: File,
) : SensorEventListener {
    private val thread = HandlerThread("sensor-$name").apply { start() }
    private val handler = Handler(thread.looper)
    private val writer = StreamWriter(dir, name)
    private val builder = ChunkBuilder(valuesPerSample)
    private var chunkStartNs = 0L

    var sampleCount = 0L
        private set

    val streamInfo: StreamInfo
        get() = StreamInfo.newBuilder().setName(name).setSensor(sensorInfo()).build()

    fun start() {
        sensorManager.registerListener(this, sensor, SAMPLING_PERIOD_US, MAX_REPORT_LATENCY_US, handler)
    }

    fun stop() {
        sensorManager.unregisterListener(this)
        handler.post {
            flush()
            writer.close()
        }
        thread.quitSafely()
        thread.join()
    }

    override fun onSensorChanged(event: SensorEvent) {
        if (builder.size == 0) chunkStartNs = event.timestamp
        builder.add(event.timestamp, event.values, event.accuracy)
        sampleCount++
        if (event.timestamp - chunkStartNs >= CHUNK_NS) flush()
    }

    override fun onAccuracyChanged(sensor: Sensor, accuracy: Int) {}

    private fun flush() {
        if (builder.size > 0) writer.append(builder.build(SystemClock.elapsedRealtimeNanos()))
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
        .setMaxReportLatencyUs(MAX_REPORT_LATENCY_US)
        .build()

    companion object {
        /** Fastest rate the sensor supports. */
        const val SAMPLING_PERIOD_US = 0

        /** No batching yet; that comes with gap detection. */
        const val MAX_REPORT_LATENCY_US = 0
        const val CHUNK_NS = 2_000_000_000L
    }
}
