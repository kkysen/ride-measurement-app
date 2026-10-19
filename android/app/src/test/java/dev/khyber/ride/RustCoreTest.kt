package dev.khyber.ride

import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test
import uniffi.ride_ffi.Fusion
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
    fun fusion() {
        Fusion(600u).use { f ->
            // Uncalibrated accelerometer values: raw, then the bias estimate.
            f.onSensorSamples(35, listOf(1L), 6u, listOf(0f, 0f, 9.8f, 0f, 0f, 0f))
            for (i in 0 until 5) f.onFix(i * 1_000_000_000L, 40.0, -74.0, 10.0, 5.0, 10.0, 0.3)
            val e = f.estimate(4_000_000_000L)
            assertEquals(10.0, e.speedMps, 0.1)
            assertTrue(Sensor.GNSS_DOPPLER in e.sensors)
            assertTrue(f.smoothed().isNotEmpty())
            assertEquals(0uL, f.gnssStats().epochsSolved)
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
