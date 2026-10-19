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
    fun usesOnlyTheFusedSpeed() {
        LiveTrack.reset()
        add(1, fused = 3.0, gps = 5.0)
        add(2, gps = 900.0)
        assertEquals(listOf(3.0, null), LiveTrack.points.value.map { it.speedMps })
    }

    @Test
    fun speedColorsAreFixedBands() {
        fun kmh(v: Double) = v / 3.6
        val rail = SpeedColors.RAIL
        assertEquals(SpeedColors.RAMP[0], rail.of(0.0))
        assertEquals(SpeedColors.RAMP[0], rail.of(kmh(9.9)))
        assertEquals(SpeedColors.RAMP[1], rail.of(kmh(10.0)))
        assertEquals(SpeedColors.RAMP[7], rail.of(kmh(120.0)))
        assertEquals(SpeedColors.RAMP[9], rail.of(kmh(190.0)))
        assertEquals(SpeedColors.RAMP[9], SpeedColors.HIGH_SPEED_RAIL.of(kmh(390.0)))
        assertEquals(SpeedColors.RAMP[9], SpeedColors.FLIGHT.of(kmh(1200.0)))
        assertEquals(SpeedColors.UNKNOWN, rail.of(null))
    }

    @Test
    fun scaleFollowsTheTopSpeed() {
        fun kmh(v: Double) = v / 3.6
        assertEquals(SpeedColors.RAIL, SpeedColors.forTopSpeed(null))
        assertEquals(SpeedColors.RAIL, SpeedColors.forTopSpeed(kmh(200.0)))
        assertEquals(SpeedColors.HIGH_SPEED_RAIL, SpeedColors.forTopSpeed(kmh(201.0)))
        assertEquals(SpeedColors.HIGH_SPEED_RAIL, SpeedColors.forTopSpeed(kmh(400.0)))
        assertEquals(SpeedColors.FLIGHT, SpeedColors.forTopSpeed(kmh(401.0)))
    }
}
