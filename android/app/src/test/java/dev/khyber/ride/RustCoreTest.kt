package dev.khyber.ride

import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test
import uniffi.ride_ffi.LiveSpeedEstimator
import uniffi.ride_ffi.Sensor
import uniffi.ride_ffi.SpeedScale
import uniffi.ride_ffi.coreVersion
import uniffi.ride_ffi.speedEdgeColorArgb
import uniffi.ride_ffi.speedScaleForTopSpeed
import uniffi.ride_ffi.trackMaxGapNs

/** Calls through the generated bindings the way the app does; the logic itself is tested in Rust. */
class RustCoreTest {
    @Test
    fun callsIntoRust() {
        assertEquals("0.1.0", coreVersion())
    }

    @Test
    fun liveSpeedEstimator() {
        LiveSpeedEstimator().use { est ->
            for (i in 0 until 5) est.onGnssFix(i * 1_000_000_000L, 0.0, 0.0, null, 10.0, 0.3)
            val e = est.estimate(4_000_000_000L)
            assertEquals(10.0, e.speedMps, 0.1)
            assertTrue(Sensor.GNSS_DOPPLER in e.sensors)
        }
    }

    @Test
    fun mapRules() {
        assertEquals(5_000_000_000L, trackMaxGapNs())
        assertEquals(SpeedScale.RAIL, speedScaleForTopSpeed(null))
        assertEquals(SpeedScale.FLIGHT, speedScaleForTopSpeed(500.0 / 3.6 * 1.0))
        // Unknown speed is gray; known speeds are opaque blues.
        assertEquals(0xFF8A8984.toUInt(), speedEdgeColorArgb(SpeedScale.RAIL, null, 10.0))
        assertEquals(0xFF.toUInt(), speedEdgeColorArgb(SpeedScale.RAIL, 10.0, 10.0) shr 24)
    }
}
