package dev.khyber.ride

import org.junit.Assert.assertEquals
import org.junit.Test
import uniffi.ride_ffi.clockTicks

/** The graph's clock times and the taps' places, through the bindings; the logic itself is tested in Rust. */
class TapMarksTest {
    private fun point(ns: Long, lat: Double, lon: Double) = LiveTrack.Point(
        ns, ns / 1_000_000, lat, lon,
        fusedSpeedMps = null, gpsSpeedMps = null, gpsSpeedAccuracyMps = null, accuracyM = null,
        startsSegment = false, smoothedSpeedMps = null, smoothedAccelMps2 = null,
    )

    @Test
    fun tapsPlacedOnThePath() {
        val path = listOf(point(0, 40.0, -74.0), point(10_000_000_000, 41.0, -73.0))
        val taps = listOf(
            TapMark(5_000_000_000, 0, "boarded"),
            TapMark(20_000_000_000, 0, "alighted"),
        )
        val placed = placeTaps(taps, path)
        assertEquals(listOf("boarded"), placed.map { it.first.label })
        assertEquals(40.5, placed[0].second.lat, 1e-9)
        assertEquals(-73.5, placed[0].second.lon, 1e-9)
    }

    @Test
    fun clockTimesInNewYork() {
        // Thu Oct 22, 2026, 6:18:02 pm in New York (EDT).
        assertEquals("6:18:02 pm", clockTime(1_792_707_482_000))
        val minute = 60_000L
        assertEquals(listOf(0L, 5 * minute, 10 * minute), clockTicks(0, 12 * minute, 0, 3u))
    }
}
