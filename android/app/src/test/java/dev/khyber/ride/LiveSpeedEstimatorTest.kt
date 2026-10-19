package dev.khyber.ride

import kotlin.math.cos
import kotlin.math.sin
import kotlin.random.Random
import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test

class LiveSpeedEstimatorTest {
    private val g = 9.80665

    /**
     * Simulates a phone at a fixed tilt in a vehicle: gravity along a tilted phone axis,
     * forward acceleration along a horizontal axis, plus vibration noise while moving.
     * [profile] gives forward acceleration (m/s^2) at time t (s).
     */
    private fun simulate(
        est: LiveSpeedEstimator,
        durationS: Double,
        profile: (Double) -> Double,
        startS: Double = 0.0,
        forwardSign: Double = 1.0,
        gnss: ((Double) -> Double?)? = null,
    ): DoubleArray {
        val rate = 400.0
        val random = Random(1)
        // Phone tilted 30 degrees about its x axis: gravity has y and z components; forward is phone x.
        val tilt = Math.toRadians(30.0)
        val down = doubleArrayOf(0.0, g * sin(tilt), g * cos(tilt))
        val forward = doubleArrayOf(forwardSign, 0.0, 0.0)
        var v = 0.0
        var t = 0.0
        while (t < durationS) {
            val a = profile(t)
            v = maxOf(0.0, v + a / rate)
            val noise = if (v > 0.1) 0.15 else 0.005
            val tNs = ((startS + t) * 1e9).toLong()
            est.onGyro(0.0, 0.0, 0.0)
            est.onAccel(
                tNs,
                down[0] + forward[0] * a + random.nextDouble(-noise, noise),
                down[1] + forward[1] * a + random.nextDouble(-noise, noise),
                down[2] + forward[2] * a + random.nextDouble(-noise, noise),
            )
            if (gnss != null && (t * rate).toLong() % rate.toLong() == 0L) {
                gnss(t)?.let { est.onGnssFix(tNs, 0.0, 0.0, null, it, 0.3) }
            }
            t += 1 / rate
        }
        return doubleArrayOf(v, startS + t)
    }

    /** Stop 5 s, accelerate 1 m/s^2 for 15 s, cruise 20 s: true speed 15 m/s. */
    private fun departAndCruise(t: Double) = when {
        t < 5 -> 0.0
        t < 20 -> 1.0
        else -> 0.0
    }

    @Test
    fun imuOnlyDepartureFromStop() {
        for (sign in listOf(1.0, -1.0)) {
            val est = LiveSpeedEstimator()
            val (trueSpeed, end) = simulate(est, 40.0, ::departAndCruise, forwardSign = sign).let { it[0] to it[1] }
            val e = est.estimate((end * 1e9).toLong())
            assertEquals(setOf(LiveSpeedEstimator.Sensor.IMU), e.sensors)
            assertEquals("forward sign $sign", trueSpeed, e.speedMps, 0.1 * trueSpeed)
        }
    }

    @Test
    fun stopResetsToZero() {
        val est = LiveSpeedEstimator()
        // Depart, cruise, brake to a stop, then stand still.
        val profile = { t: Double ->
            when {
                t < 5 -> 0.0
                t < 20 -> 1.0
                t < 30 -> 0.0
                t < 45 -> -1.0
                else -> 0.0
            }
        }
        val end = simulate(est, 55.0, profile)[1]
        val e = est.estimate((end * 1e9).toLong())
        assertTrue(e.stopped)
        assertTrue(LiveSpeedEstimator.Sensor.STOP in e.sensors)
        assertEquals(0.0, e.speedMps, 0.1)
    }

    @Test
    fun gnssUsedWhenFreshThenImuCarriesOn() {
        val est = LiveSpeedEstimator()
        // GNSS for the first 30 s (true speed), then lost; keep cruising, then accelerate again.
        val profile = { t: Double ->
            when {
                t < 5 -> 0.0
                t < 15 -> 1.0
                t < 40 -> 0.0
                t < 45 -> 1.0
                else -> 0.0
            }
        }
        var v = 0.0
        var last = 0.0
        val gnss = { t: Double ->
            // Integrate the same profile to know the true speed at t.
            while (last < t) {
                v += profile(last) / 400.0
                last += 1 / 400.0
            }
            if (t < 30) v else null
        }
        val (trueSpeed, end) = simulate(est, 50.0, profile, forwardSign = -1.0, gnss = gnss).let { it[0] to it[1] }
        val e = est.estimate((end * 1e9).toLong())
        assertEquals(setOf(LiveSpeedEstimator.Sensor.IMU), e.sensors)
        assertTrue("since GNSS ${e.sinceGnssS}", e.sinceGnssS!! > 15)
        assertEquals(trueSpeed, e.speedMps, 0.1 * trueSpeed)
    }

    @Test
    fun rotationOfThePhoneIsTrackedByTheGyro() {
        val est = LiveSpeedEstimator()
        val end = simulate(est, 30.0, ::departAndCruise)[1]
        val before = est.estimate((end * 1e9).toLong()).speedMps
        // Rotate the phone 90 degrees about gravity (z after removing tilt is complex; use the tilted down axis) over 2 s.
        val rate = 400.0
        val tilt = Math.toRadians(30.0)
        val axis = doubleArrayOf(0.0, sin(tilt), cos(tilt))
        val w = Math.PI / 2 / 2.0
        var angle = 0.0
        var t = end
        while (angle < Math.PI / 2) {
            angle += w / rate
            t += 1 / rate
            // Forward (phone x at angle 0) seen in the rotating phone frame rotates by -angle about the down axis.
            est.onGyro(axis[0] * w, axis[1] * w, axis[2] * w)
            est.onAccel((t * 1e9).toLong(), 0.0 + 0.1 * sin(t * 50), 9.80665 * sin(tilt), 9.80665 * cos(tilt))
        }
        val after = est.estimate((t * 1e9).toLong()).speedMps
        assertEquals(before, after, 0.1 * before)
    }

    @Test
    fun wrongGnssSpikeIsRejected() {
        val est = LiveSpeedEstimator()
        val profile = { t: Double -> if (t in 5.0..15.0) 1.0 else 0.0 }
        var v = 0.0
        var last = 0.0
        val gnss = { t: Double ->
            while (last < t) {
                v += profile(last) / 400.0
                last += 1 / 400.0
            }
            // One wildly wrong fix at 25 s, as from multipath in a station.
            if (t.toInt() == 25) v + 15 else v
        }
        val (trueSpeed, end) = simulate(est, 26.5, profile, gnss = gnss).let { it[0] to it[1] }
        val e = est.estimate((end * 1e9).toLong())
        assertEquals(1, e.rejected[LiveSpeedEstimator.Sensor.GNSS_DOPPLER])
        assertEquals(trueSpeed, e.speedMps, 0.5)
    }

    @Test
    fun sustainedGnssDisagreementReinitializesFromGnss() {
        val est = LiveSpeedEstimator()
        simulate(est, 20.0, { t -> if (t in 5.0..10.0) 1.0 else 0.0 })
        // The IMU thinks ~5 m/s; GNSS consistently says 20 (e.g. the IMU estimate drifted). GNSS wins.
        for (i in 0 until 8) est.onGnssFix(((21 + i) * 1e9).toLong(), 0.0, 0.0, null, 20.0, 0.2)
        assertEquals(20.0, est.estimate(29_000_000_000L).speedMps, 1.0)
    }

    @Test
    fun positionDifferencedSpeedIsFused() {
        val est = LiveSpeedEstimator()
        // 10 m/s east along the equator: 1 degree of longitude is ~111.2 km.
        for (i in 0 until 10) {
            val lon = i * 10.0 / 111_195.0
            est.onGnssFix((i * 1e9).toLong(), 0.0, lon, 1.0, null, null)
        }
        val e = est.estimate(9_000_000_000L)
        assertEquals(10.0, e.speedMps, 0.3)
        assertTrue(LiveSpeedEstimator.Sensor.GNSS_POSITION in e.sensors)
    }

    @Test
    fun impossibleSpeedsAreRejectedEvenFirst() {
        val est = LiveSpeedEstimator()
        // A glitched first fix, then a run of them, as could otherwise force a re-initialization.
        est.onGnssFix(0L, 0.0, 0.0, null, 500.0, 0.3)
        for (i in 1 until 10) est.onGnssFix(i * 1_000_000_000L, 0.0, 0.0, null, if (i < 3) 10.0 else 900.0, 0.3)
        val e = est.estimate(9_000_000_000L)
        assertEquals(8, e.rejected[LiveSpeedEstimator.Sensor.GNSS_DOPPLER])
        assertEquals(10.0, e.speedMps, 1.0)
    }
}
