package dev.khyber.ride

import android.location.Location
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import uniffi.ride_ffi.SmoothedSpeed
import uniffi.ride_ffi.trackMaxGapNs

/**
 * The current recording's GPS path, for the live map.
 *
 * Only GPS-provider fixes, as in `crates/session/src/track.rs`: fused fixes can be network positions,
 * which can be the wrong station underground.
 * Kept after a recording stops, so the map still shows it, until the next one starts.
 */
object LiveTrack {
    data class Point(
        val elapsedRealtimeNs: Long,
        val timeMs: Long,
        val lat: Double,
        val lon: Double,
        /**
         * The live estimate fusing IMU, GNSS, and stops, if there is one; what the map is colored by.
         * Never the raw GPS speed, which can spike.
         */
        val fusedSpeedMps: Double?,
        val gpsSpeedMps: Double?,
        val gpsSpeedAccuracyMps: Double?,
        val accuracyM: Double?,
        /** True if this fix starts a new segment after a gap (a tunnel), so no line is drawn to it. */
        val startsSegment: Boolean,
        /** The smoothed speed (forward and backward passes over recent input), once smoothing has reached it. */
        val smoothedSpeedMps: Double? = null,
    ) {
        /** What the map is colored by: smoothed if there is one, else live. */
        val speedMps: Double? get() = smoothedSpeedMps ?: fusedSpeedMps
    }

    private val _points = MutableStateFlow<List<Point>>(emptyList())
    val points: StateFlow<List<Point>> = _points

    fun reset() {
        _points.value = emptyList()
    }

    /** Called from the GPS listener; never throws, so the map can't break recording. */
    fun onGpsLocation(location: Location) {
        try {
            add(
                location.elapsedRealtimeNanos,
                location.time,
                location.latitude,
                location.longitude,
                LiveSpeed.estimate.value?.speedMps,
                if (location.hasSpeed()) location.speed.toDouble() else null,
                if (location.hasSpeedAccuracy()) location.speedAccuracyMetersPerSecond.toDouble() else null,
                if (location.hasAccuracy()) location.accuracy.toDouble() else null,
            )
        } catch (e: Throwable) {
            Health.report("live track", e)
        }
    }

    /** Replaces recent points' speeds with smoothed ones at the same fix times. */
    @Synchronized
    fun applySmoothed(smoothed: List<SmoothedSpeed>) {
        if (smoothed.isEmpty()) return
        val byTime = smoothed.associate { it.elapsedRealtimeNs to it.speedMps }
        val points = _points.value
        var changed = false
        val updated = points.map { p ->
            val s = byTime[p.elapsedRealtimeNs]
            if (s != null && s != p.smoothedSpeedMps) {
                changed = true
                p.copy(smoothedSpeedMps = s)
            } else {
                p
            }
        }
        if (changed) _points.value = updated
    }

    @Synchronized
    fun add(
        elapsedRealtimeNs: Long,
        timeMs: Long,
        lat: Double,
        lon: Double,
        fusedSpeedMps: Double?,
        gpsSpeedMps: Double?,
        gpsSpeedAccuracyMps: Double?,
        accuracyM: Double?,
    ) {
        val points = _points.value
        val last = points.lastOrNull()
        if (last != null && elapsedRealtimeNs <= last.elapsedRealtimeNs) return
        val startsSegment = last == null || elapsedRealtimeNs - last.elapsedRealtimeNs > maxGapNs
        _points.value = points + Point(
            elapsedRealtimeNs, timeMs, lat, lon, fusedSpeedMps, gpsSpeedMps, gpsSpeedAccuracyMps, accuracyM, startsSegment,
        )
    }

    /** From `crates/session/src/track.rs`, the same rule as `ride map`. */
    private val maxGapNs by lazy { trackMaxGapNs() }
}
