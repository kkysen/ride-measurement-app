package dev.khyber.ride

import android.location.Location
import android.util.Log
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow

/**
 * The current recording's GPS path, for the live map.
 *
 * Only GPS-provider fixes, like `crates/session/src/track.rs`: fused fixes can be network positions,
 * which can be the wrong station underground.
 * Kept after a recording stops, so the map still shows it, until the next one starts.
 */
object LiveTrack {
    data class Point(
        val elapsedRealtimeNs: Long,
        val timeMs: Long,
        val lat: Double,
        val lon: Double,
        /** The live estimate fusing IMU, GNSS, and stops, if there is one; what the map is colored by. */
        val fusedSpeedMps: Double?,
        val gpsSpeedMps: Double?,
        val gpsSpeedAccuracyMps: Double?,
        val accuracyM: Double?,
        /** True if this fix starts a new segment after a gap (a tunnel), so no line is drawn to it. */
        val startsSegment: Boolean,
    ) {
        val speedMps: Double? get() = fusedSpeedMps ?: gpsSpeedMps
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
            Log.e("RideLogger", "live track failed", e)
        }
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
        val startsSegment = last == null || elapsedRealtimeNs - last.elapsedRealtimeNs > MAX_GAP_NS
        _points.value = points + Point(
            elapsedRealtimeNs, timeMs, lat, lon, fusedSpeedMps, gpsSpeedMps, gpsSpeedAccuracyMps, accuracyM, startsSegment,
        )
    }

    /** Same as `MAX_GAP_NS` in `crates/session/src/track.rs`. */
    const val MAX_GAP_NS = 5_000_000_000L
}

/** The map's speed colors: the same light-to-dark blue ramp as `ride map`, over a fixed 0 to [MAX_KMH] km/h. */
object SpeedColors {
    val RAMP = intArrayOf(
        0xFF86B6EF.toInt(), 0xFF6DA7EC.toInt(), 0xFF5598E7.toInt(), 0xFF3987E5.toInt(), 0xFF2A78D6.toInt(),
        0xFF256ABF.toInt(), 0xFF1C5CAB.toInt(), 0xFF184F95.toInt(), 0xFF104281.toInt(), 0xFF0D366B.toInt(),
    )
    const val UNKNOWN = 0xFF8A8984.toInt()
    const val MAX_KMH = 100.0

    fun of(speedMps: Double?): Int {
        if (speedMps == null) return UNKNOWN
        val f = (speedMps * 3.6 / MAX_KMH).coerceIn(0.0, 1.0) * (RAMP.size - 1)
        return RAMP[Math.round(f).toInt()]
    }
}
