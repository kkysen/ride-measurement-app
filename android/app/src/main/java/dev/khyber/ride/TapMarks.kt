package dev.khyber.ride

import android.util.Log
import androidx.compose.runtime.Composable
import androidx.compose.runtime.remember
import java.time.Instant
import java.time.ZoneId
import uniffi.ride_ffi.RideClock
import uniffi.ride_ffi.positionsAt

/** A tap to show on the graph or the map: when (elapsed realtime ns, and on the clock), and what it was. */
data class TapMark(val ns: Long, val unixMs: Long, val label: String)

/**
 * Clock times in the phone's zone now, for the ride being recorded (Rust's `RideClock::live`);
 * the offset now is only for a zone Rust's database doesn't know.
 */
fun liveClock(): RideClock {
    val zone = ZoneId.systemDefault()
    return RideClock.live(zone.id, zone.rules.getOffset(Instant.now()).totalSeconds * 1000L)
}

/** [liveClock], made again when the phone's zone changes (as on a flight). */
@Composable
fun rememberLiveClock(): RideClock {
    val id = ZoneId.systemDefault().id
    return remember(id) { liveClock() }
}

/**
 * A past ride's clock times, in its own zone (Rust's `RideClock::for_session`), or the phone's now if its
 * session can't be read. Reads the ride's events and location fixes, so not on the main thread.
 */
fun sessionClock(sessionDir: String): RideClock =
    runCatching { RideClock.forSession(sessionDir) }
        .onFailure { Log.e("RideLogger", "reading the ride's time zone failed", it) }
        .getOrElse { liveClock() }

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
