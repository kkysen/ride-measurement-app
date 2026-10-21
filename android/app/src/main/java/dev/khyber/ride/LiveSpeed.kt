package dev.khyber.ride

import android.location.Location
import android.os.Handler
import android.os.HandlerThread
import android.os.SystemClock
import android.util.Log
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import uniffi.ride_ffi.Estimate
import uniffi.ride_ffi.Fusion
import uniffi.ride_ffi.GnssStats
import uniffi.ride_ffi.settleNs
import uniffi.ride_ffi.smoothEveryNs

/**
 * Runs the Rust fusion (`crates/fusion/src/fusion.rs`, the same code offline tools replay) on its own thread
 * during a recording: IMU, GPS fixes, raw GNSS, and navigation messages go in as Android delivers them,
 * and the live estimate, raw GNSS progress, and the smoothed recent track (re-run in Rust every [smoothEveryNs]) come out.
 * Recording doesn't depend on it: if the Rust library fails, [error] says why and there's no live speed.
 */
object LiveSpeed {
    private var thread: HandlerThread? = null
    private var handler: Handler? = null
    private var fusion: Fusion? = null
    private val _estimate = MutableStateFlow<Estimate?>(null)
    val estimate: StateFlow<Estimate?> = _estimate
    private val _gnss = MutableStateFlow<GnssStats?>(null)
    val gnss: StateFlow<GnssStats?> = _gnss
    private val _error = MutableStateFlow<String?>(null)
    val error: StateFlow<String?> = _error

    private var lateInputs = 0uL

    private val publish = object : Runnable {
        override fun run() {
            safely {
                _estimate.value = fusion?.estimate(SystemClock.elapsedRealtimeNanos())
                _gnss.value = fusion?.gnssStats()
                val late = fusion?.lateInputs() ?: 0uL
                // These went into the live forward pass out of time order, so live differs from a replay there.
                if (late > lateInputs) {
                    val ms = settleNs() / 1_000_000
                    Health.report("live speed", IllegalStateException("$late inputs so far arrived over $ms ms late"))
                }
                lateInputs = late
            }
            handler?.postDelayed(this, PUBLISH_MS)
        }
    }

    // Smoothing re-runs the window backward, so after a tunnel the map's recent path is recolored with the speed
    // the first fix after it implies.
    private val smooth = object : Runnable {
        override fun run() {
            safely { fusion?.smoothed()?.let { LiveTrack.applySmoothed(it) } }
            handler?.postDelayed(this, SMOOTH_MS)
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
        lateInputs = 0uL
        safely { fusion = Fusion() }
        handler?.post(publish)
        handler?.postDelayed(smooth, SMOOTH_MS)
    }

    @Synchronized
    fun stop() {
        thread?.quitSafely()
        thread = null
        handler = null
        fusion?.let { safely { it.close() } }
        fusion = null
        _estimate.value = null
        _gnss.value = null
    }

    /** A batch of one sensor's samples, from its recorder's thread. Rust decides what to use. */
    fun onSensorSamples(type: Int, timestampsNs: List<Long>, valuesPerSample: Int, values: List<Float>) {
        val h = handler ?: return
        h.post { safely { fusion?.onSensorSamples(type, timestampsNs, valuesPerSample.toUInt(), values) } }
    }

    /** GPS-provider fixes only: fused fixes may mix in network location, which can be wrong underground. */
    fun onGpsLocation(location: Location) {
        val h = handler ?: return
        val t = location.elapsedRealtimeNanos
        val lat = location.latitude
        val lon = location.longitude
        val altitude = if (location.hasAltitude()) location.altitude else null
        val accuracy = if (location.hasAccuracy()) location.accuracy.toDouble() else null
        val speed = if (location.hasSpeed()) location.speed.toDouble() else null
        val speedAccuracy = if (location.hasSpeedAccuracy()) location.speedAccuracyMetersPerSecond.toDouble() else null
        h.post { safely { fusion?.onFix(t, lat, lon, altitude, accuracy, speed, speedAccuracy) } }
    }

    /** An encoded `GnssMeasurementsEvent`, as recorded. */
    fun onGnssMeasurements(receivedNs: Long, encoded: ByteArray) {
        val h = handler ?: return
        h.post { safely { fusion?.onGnssMeasurements(receivedNs, encoded) } }
    }

    /** An encoded `GnssNavigationMessage`, as recorded. */
    fun onNavigationMessage(encoded: ByteArray) {
        val h = handler ?: return
        h.post { safely { fusion?.onNavigationMessage(encoded) } }
    }

    /** A downloaded orbit file ([OrbitFetcher]); a bad one is reported and live speed carries on without it. */
    fun addOrbitFile(path: String) {
        val h = handler ?: return
        h.post { Health.guard("orbit file") { fusion?.addOrbitFile(path) } }
    }

    /** Every orbit file already in a session's `orbits/`, e.g. after a restart. */
    fun addSessionOrbits(sessionDir: String) {
        val h = handler ?: return
        h.post { Health.guard("session orbits") { fusion?.addSessionOrbits(sessionDir) } }
    }

    private const val PUBLISH_MS = 500L

    /** How often the map picks up the latest smoothing: as often as Rust re-runs it. */
    private val SMOOTH_MS = smoothEveryNs() / 1_000_000
}
