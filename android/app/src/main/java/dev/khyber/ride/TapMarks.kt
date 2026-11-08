package dev.khyber.ride

import java.time.Instant
import java.time.ZoneId
import java.time.format.DateTimeFormatter
import java.util.Locale
import uniffi.ride_ffi.positionsAt

/** A tap to show on the graph or the map: when (elapsed realtime ns, and on the clock), and what it was. */
data class TapMark(val ns: Long, val unixMs: Long, val label: String)

private val TAP_TIME = DateTimeFormatter.ofPattern("h:mm:ss a", Locale.US).withZone(ZoneId.of("America/New_York"))

/** A clock time as the app shows it: 12-hour, New York, like 6:18:30 pm. */
fun clockTime(unixMs: Long): String = TAP_TIME.format(Instant.ofEpochMilli(unixMs)).lowercase()

/** A tap between path points further apart than this (ns) gets no place on the map: no telling where the rider was. */
private const val MAX_TAP_GAP_NS = 5 * 60_000_000_000L

/**
 * Where on `path` (in time order) each tap was, from Rust (`positions_at`); taps off the path are left out.
 * Only the path from just before the first tap is passed, so the live map's recent taps cost little.
 */
fun placeTaps(taps: List<TapMark>, path: List<LiveTrack.Point>): List<Pair<TapMark, LiveTrack.Point>> {
    if (taps.isEmpty() || path.isEmpty()) return emptyList()
    val first = taps.minOf { it.ns }
    val from = (path.indexOfLast { it.elapsedRealtimeNs < first }).coerceAtLeast(0)
    val part = path.subList(from, path.size)
    val at = positionsAt(
        taps.map { it.ns },
        part.map { it.elapsedRealtimeNs },
        part.map { it.lat },
        part.map { it.lon },
        MAX_TAP_GAP_NS,
    )
    return taps.zip(at).mapNotNull { (tap, p) ->
        p?.let {
            tap to LiveTrack.Point(
                tap.ns, tap.unixMs, it.lat, it.lon,
                fusedSpeedMps = null, gpsSpeedMps = null, gpsSpeedAccuracyMps = null, accuracyM = null,
                startsSegment = false, smoothedSpeedMps = null, smoothedAccelMps2 = null,
            )
        }
    }
}
