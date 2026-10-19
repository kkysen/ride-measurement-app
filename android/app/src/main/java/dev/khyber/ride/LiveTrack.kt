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
    ) {
        val speedMps: Double? get() = fusedSpeedMps
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

/**
 * The map's speed colors: the same light-to-dark blue ramp and bands as `ride map`.
 * Bands are narrow at low speeds and wide at high ones, and fixed, so a color means the same speed on every ride.
 * Each scale covers its kind of ride's top speed, and a ride uses the first whose limit its top speed is within:
 * subways and commuter rail up to 200 km/h, high-speed rail up to 400 (CR450), flights up to Mach 1.
 */
enum class SpeedColors(
    val label: String,
    val maxKmh: Double,
    /** Lower edge of each band, km/h; the last band is open-ended. */
    val bandsKmh: IntArray,
) {
    RAIL("rail", 200.0, intArrayOf(0, 10, 20, 30, 40, 60, 80, 100, 130, 160)),
    HIGH_SPEED_RAIL("high-speed rail", 400.0, intArrayOf(0, 25, 50, 100, 150, 200, 250, 300, 340, 380)),
    FLIGHT("flight", Double.POSITIVE_INFINITY, intArrayOf(0, 50, 100, 200, 300, 400, 500, 650, 800, 950)),
    ;

    fun of(speedMps: Double?): Int {
        if (speedMps == null) return UNKNOWN
        val kmh = speedMps * 3.6
        return RAMP[bandsKmh.indexOfLast { kmh >= it }.coerceAtLeast(0)]
    }

    companion object {
        val RAMP = intArrayOf(
            0xFF86B6EF.toInt(), 0xFF6DA7EC.toInt(), 0xFF5598E7.toInt(), 0xFF3987E5.toInt(), 0xFF2A78D6.toInt(),
            0xFF256ABF.toInt(), 0xFF1C5CAB.toInt(), 0xFF184F95.toInt(), 0xFF104281.toInt(), 0xFF0D366B.toInt(),
        )
        const val UNKNOWN = 0xFF8A8984.toInt()

        fun forTopSpeed(maxSpeedMps: Double?): SpeedColors =
            entries.first { (maxSpeedMps ?: 0.0) * 3.6 <= it.maxKmh }
    }
}
