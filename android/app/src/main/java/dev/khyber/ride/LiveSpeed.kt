package dev.khyber.ride

import android.hardware.Sensor
import android.hardware.SensorEvent
import android.location.Location
import android.os.Handler
import android.os.HandlerThread
import android.os.SystemClock
import android.util.Log
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import uniffi.ride_ffi.Estimate
import uniffi.ride_ffi.LiveSpeedEstimator

/**
 * Runs the Rust live speed estimator (`crates/fusion/src/live_speed.rs`, the same code offline tools replay)
 * on its own thread during a recording, and publishes the estimate for the UI.
 * Recording doesn't depend on it: if the Rust library fails, [error] says why and there's no live speed.
 */
object LiveSpeed {
    private var thread: HandlerThread? = null
    private var handler: Handler? = null
    private var estimator: LiveSpeedEstimator? = null
    private val _estimate = MutableStateFlow<Estimate?>(null)
    val estimate: StateFlow<Estimate?> = _estimate
    private val _error = MutableStateFlow<String?>(null)
    val error: StateFlow<String?> = _error

    private val publish = object : Runnable {
        override fun run() {
            safely { _estimate.value = estimator?.estimate(SystemClock.elapsedRealtimeNanos()) }
            handler?.postDelayed(this, PUBLISH_MS)
        }
    }

    /** Runs a call into Rust; a failure stops live speed instead of crashing the thread and the app with it. */
    private inline fun safely(block: () -> Unit) {
        if (_error.value != null) return
        try {
            block()
        } catch (e: Throwable) {
            Log.e("RideLogger", "live speed failed", e)
            _error.value = "Live speed failed (recording still works):\n${Log.getStackTraceString(e)}"
            _estimate.value = null
        }
    }

    @Synchronized
    fun start() {
        if (thread != null) return
        val t = HandlerThread("live-speed").apply { start() }
        thread = t
        handler = Handler(t.looper)
        safely { estimator = LiveSpeedEstimator() }
        handler?.post(publish)
    }

    @Synchronized
    fun stop() {
        thread?.quitSafely()
        thread = null
        handler = null
        estimator?.let { safely { it.close() } }
        estimator = null
        _estimate.value = null
    }

    /** Called on a sensor thread; copies what's needed and hands it to the estimator thread. */
    fun onSensorEvent(event: SensorEvent) {
        val h = handler ?: return
        val values = event.values
        // Uncalibrated sensors carry the bias estimate in values[3..5]; subtract it.
        val bias = event.sensor.type == Sensor.TYPE_ACCELEROMETER_UNCALIBRATED ||
            event.sensor.type == Sensor.TYPE_GYROSCOPE_UNCALIBRATED
        val x = values[0] - if (bias) values[3] else 0f
        val y = values[1] - if (bias) values[4] else 0f
        val z = values[2] - if (bias) values[5] else 0f
        val t = event.timestamp
        when (event.sensor.type) {
            Sensor.TYPE_ACCELEROMETER, Sensor.TYPE_ACCELEROMETER_UNCALIBRATED ->
                h.post { safely { estimator?.onAccel(t, x.toDouble(), y.toDouble(), z.toDouble()) } }
            Sensor.TYPE_GYROSCOPE, Sensor.TYPE_GYROSCOPE_UNCALIBRATED ->
                h.post { safely { estimator?.onGyro(x.toDouble(), y.toDouble(), z.toDouble()) } }
        }
    }

    /** GPS-provider fixes only: fused fixes may mix in network location, which can be wrong underground. */
    fun onGpsLocation(location: Location) {
        val h = handler ?: return
        val t = location.elapsedRealtimeNanos
        val lat = location.latitude
        val lon = location.longitude
        val accuracy = if (location.hasAccuracy()) location.accuracy.toDouble() else null
        val speed = if (location.hasSpeed()) location.speed.toDouble() else null
        val speedAccuracy = if (location.hasSpeedAccuracy()) location.speedAccuracyMetersPerSecond.toDouble() else null
        h.post { safely { estimator?.onGnssFix(t, lat, lon, accuracy, speed, speedAccuracy) } }
    }

    private const val PUBLISH_MS = 500L
}
