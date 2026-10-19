package dev.khyber.ride

import org.junit.Assert.assertEquals
import org.junit.Test

class LiveTrackTest {
    private fun add(tS: Long, fused: Double? = null, gps: Double? = null) =
        LiveTrack.add(tS * 1_000_000_000, tS * 1000, 40.0 + tS * 1e-4, -74.0, fused, gps, null, null)

    @Test
    fun splitsAtGapsAndIgnoresStaleFixes() {
        LiveTrack.reset()
        add(1)
        add(2)
        add(2) // duplicate
        add(1) // out of order
        add(10) // after a tunnel-length gap
        add(11)
        val points = LiveTrack.points.value
        assertEquals(listOf(1L, 2L, 10L, 11L), points.map { it.timeMs / 1000 })
        assertEquals(listOf(true, false, true, false), points.map { it.startsSegment })
    }

    @Test
    fun prefersTheFusedSpeed() {
        LiveTrack.reset()
        add(1, fused = 3.0, gps = 5.0)
        add(2, gps = 5.0)
        assertEquals(listOf(3.0, 5.0), LiveTrack.points.value.map { it.speedMps })
    }

    @Test
    fun speedColorsRunLightToDarkAndClamp() {
        assertEquals(SpeedColors.RAMP.first(), SpeedColors.of(0.0))
        assertEquals(SpeedColors.RAMP.last(), SpeedColors.of(SpeedColors.MAX_KMH / 3.6))
        assertEquals(SpeedColors.RAMP.last(), SpeedColors.of(1000.0))
        assertEquals(SpeedColors.UNKNOWN, SpeedColors.of(null))
    }
}
