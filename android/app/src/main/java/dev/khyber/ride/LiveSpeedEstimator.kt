package dev.khyber.ride

import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.asin
import kotlin.math.cos
import kotlin.math.exp
import kotlin.math.sin
import kotlin.math.sqrt

/**
 * Live speed for display; offline processing recomputes speed properly from the raw data.
 *
 * A 1-D Kalman filter along the direction of travel, with state (speed, accelerometer bias along that direction):
 * - Prediction: acceleration along the travel axis from the IMU. The axis is the dominant horizontal acceleration
 *   direction in the phone's frame; gravity and the axis are carried through phone rotations by the gyro,
 *   and gravity is re-learned from the accelerometer mostly while stopped, so sustained acceleration isn't taken for tilt.
 * - Measurements, each weighted by its own uncertainty:
 *   GNSS Doppler speed, speed from successive GNSS positions, and stops (speed 0, from a quiet accelerometer).
 * - Every measurement is gated: one more than [GATE_SIGMA] sigma from the prediction is rejected and counted,
 *   so a single wrong sensor can't drag the estimate. If GNSS is rejected [GNSS_REINIT_REJECTS] times in a row,
 *   the IMU side is what's wrong, and the filter re-initializes from GNSS.
 * - The travel axis's sign (which way is forward) comes from GNSS speed changes, or from a departure from a stop
 *   (a train or bus accelerates forward when leaving). Until it's known, speed is the magnitude of the integral.
 *
 * Not thread-safe; feed it from one thread.
 */
class LiveSpeedEstimator {
    enum class Sensor { IMU, GNSS_DOPPLER, GNSS_POSITION, STOP }

    data class Estimate(
        val speedMps: Double,
        val sigmaMps: Double,
        /** Sensors that contributed in the last [RECENT_S] seconds. */
        val sensors: Set<Sensor>,
        val stopped: Boolean,
        val sinceGnssS: Double?,
        /** Measurements rejected as outliers so far, per sensor. */
        val rejected: Map<Sensor, Int>,
    )

    // IMU front end.
    private val gravity = DoubleArray(3)
    private var gravityInit = false
    private val cov = DoubleArray(9)
    private val axis = doubleArrayOf(1.0, 0.0, 0.0)
    private var axisKnown = false
    private var axisSignKnown = false
    private val omega = DoubleArray(3)
    private var lastNs = 0L
    private var magMean = 0.0
    private var magVar = 0.0
    private var stillSinceNs = 0L
    private var stopped = false
    private var sinceStopAlong = 0.0

    // Filter state: signed speed along the axis, and along-axis accelerometer bias.
    private var v = 0.0
    private var b = 0.0
    private var p00 = 1.0
    private var p01 = 0.0
    private var p11 = BIAS_SIGMA0 * BIAS_SIGMA0
    private var initialized = false

    // GNSS.
    private var lastGnssNs = 0L
    private var prevDopplerSpeed = Double.NaN
    private var alongSinceGnss = 0.0
    private var signEvidence = 0.0
    private var gnssRejectStreak = 0
    private var prevFix: Fix? = null

    private val lastUsedNs = HashMap<Sensor, Long>()
    private val rejected = HashMap<Sensor, Int>()

    private data class Fix(val tNs: Long, val lat: Double, val lon: Double, val accuracyM: Double)

    /** Gyro sample (rad/s), bias already removed. */
    fun onGyro(wx: Double, wy: Double, wz: Double) {
        omega[0] = wx
        omega[1] = wy
        omega[2] = wz
    }

    /** Accelerometer sample with gravity included (m/s^2), bias already removed, on the elapsed realtime clock. */
    fun onAccel(tNs: Long, ax: Double, ay: Double, az: Double) {
        if (!gravityInit) {
            gravity[0] = ax
            gravity[1] = ay
            gravity[2] = az
            gravityInit = true
        }
        if (lastNs == 0L || tNs <= lastNs) {
            lastNs = maxOf(lastNs, tNs)
            return
        }
        val dt = minOf((tNs - lastNs) * 1e-9, MAX_DT_S)
        lastNs = tNs

        // Stillness from the variance of |a|.
        val mag = sqrt(ax * ax + ay * ay + az * az)
        val kMag = 1 - exp(-dt / STILL_WINDOW_S)
        val d = mag - magMean
        magMean += kMag * d
        magVar = (1 - kMag) * (magVar + kMag * d * d)
        val still = sqrt(magVar) < STILL_STD_MPS2
        if (!still) stillSinceNs = 0L else if (stillSinceNs == 0L) stillSinceNs = tNs

        // World-fixed vectors seen from a rotating phone turn the other way: v' = v - (omega x v) dt.
        rotateWorldFixed(gravity, dt)
        rotateWorldFixed(axis, dt)
        rotateCov(dt)
        val kG = 1 - exp(-dt / if (still) GRAVITY_TAU_STILL_S else GRAVITY_TAU_MOVING_S)
        gravity[0] += kG * (ax - gravity[0])
        gravity[1] += kG * (ay - gravity[1])
        gravity[2] += kG * (az - gravity[2])
        val gNorm = sqrt(gravity[0] * gravity[0] + gravity[1] * gravity[1] + gravity[2] * gravity[2])
        if (gNorm < 1e-6) return
        val gx = gravity[0] / gNorm
        val gy = gravity[1] / gNorm
        val gz = gravity[2] / gNorm
        val lx = ax - gravity[0]
        val ly = ay - gravity[1]
        val lz = az - gravity[2]
        val vertical = lx * gx + ly * gy + lz * gz
        val hx = lx - vertical * gx
        val hy = ly - vertical * gy
        val hz = lz - vertical * gz
        if (!still) updateAxis(hx, hy, hz, dt)
        val along = if (axisKnown) hx * axis[0] + hy * axis[1] + hz * axis[2] else 0.0

        predict(along, dt)
        markUsed(Sensor.IMU, tNs)
        alongSinceGnss += along * dt

        val wasStopped = stopped
        stopped = stillSinceNs != 0L && (tNs - stillSinceNs) * 1e-9 >= STILL_S
        if (stopped) {
            update(0.0, STOP_SIGMA_MPS * STOP_SIGMA_MPS, Sensor.STOP, tNs)
            sinceStopAlong = 0.0
        } else {
            if (wasStopped) sinceStopAlong = 0.0
            sinceStopAlong += along * dt
            // A departure accelerates forward, which fixes the axis sign if GNSS hasn't.
            if (!axisSignKnown && axisKnown && abs(sinceStopAlong) > DEPARTURE_EVIDENCE_MPS) {
                if (sinceStopAlong < 0) flipAxis()
                axisSignKnown = true
            }
        }
    }

    /** A GNSS fix: Doppler speed if the fix has one, and position for position-differenced speed. */
    fun onGnssFix(
        tNs: Long,
        latDeg: Double,
        lonDeg: Double,
        horizontalAccuracyM: Double?,
        speedMps: Double?,
        speedAccuracyMps: Double?,
    ) {
        lastGnssNs = tNs
        if (speedMps != null) onDopplerSpeed(tNs, speedMps, speedAccuracyMps ?: DEFAULT_DOPPLER_SIGMA_MPS)
        if (horizontalAccuracyM != null) {
            val fix = Fix(tNs, latDeg, lonDeg, horizontalAccuracyM)
            prevFix?.let { prev ->
                val dt = (tNs - prev.tNs) * 1e-9
                if (dt in 0.5..POSITION_MAX_DT_S) {
                    val speed = distanceM(prev, fix) / dt
                    val sigma = sqrt(prev.accuracyM * prev.accuracyM + fix.accuracyM * fix.accuracyM) / dt
                    if (sigma < POSITION_MAX_SIGMA_MPS) measureSpeedMagnitude(speed, sigma, Sensor.GNSS_POSITION, tNs)
                }
            }
            prevFix = fix
        }
    }

    fun estimate(nowNs: Long): Estimate = Estimate(
        speedMps = abs(v),
        sigmaMps = sqrt(maxOf(p00, 0.0)),
        sensors = lastUsedNs.filterValues { (nowNs - it) * 1e-9 < RECENT_S }.keys,
        stopped = stopped,
        sinceGnssS = if (lastGnssNs == 0L) null else (nowNs - lastGnssNs) * 1e-9,
        rejected = rejected.toMap(),
    )

    private fun onDopplerSpeed(tNs: Long, speed: Double, sigma: Double) {
        // Speed change between fixes vs integrated along-axis acceleration fixes the axis sign.
        if (!prevDopplerSpeed.isNaN() && axisKnown && !axisSignKnown) {
            val dv = speed - prevDopplerSpeed
            if (abs(dv) > 0.3) {
                signEvidence += dv * alongSinceGnss
                if (abs(signEvidence) > SIGN_EVIDENCE) {
                    if (signEvidence < 0) flipAxis()
                    axisSignKnown = true
                }
            }
        }
        prevDopplerSpeed = speed
        alongSinceGnss = 0.0
        measureSpeedMagnitude(speed, maxOf(sigma, MIN_DOPPLER_SIGMA_MPS), Sensor.GNSS_DOPPLER, tNs)
    }

    /** GNSS measures speed magnitude; the filter's speed is signed along the axis, so match the sign. */
    private fun measureSpeedMagnitude(speed: Double, sigma: Double, sensor: Sensor, tNs: Long) {
        val signed = if (v < 0 && !axisSignKnown) -speed else speed
        if (!initialized) {
            v = signed
            p00 = sigma * sigma
            initialized = true
            markUsed(sensor, tNs)
            return
        }
        val accepted = update(signed, sigma * sigma, sensor, tNs)
        if (accepted) {
            gnssRejectStreak = 0
        } else if (++gnssRejectStreak >= GNSS_REINIT_REJECTS) {
            // Several GNSS fixes in a row disagree with the IMU-driven prediction: trust GNSS.
            v = signed
            b = 0.0
            p00 = sigma * sigma
            p01 = 0.0
            p11 = BIAS_SIGMA0 * BIAS_SIGMA0
            gnssRejectStreak = 0
            markUsed(sensor, tNs)
        }
    }

    private fun predict(along: Double, dt: Double) {
        if (axisSignKnown || stopped) {
            v += (along - b) * dt
        } else {
            v += along * dt
        }
        // P = F P F^T + Q with F = [[1, -dt], [0, 1]].
        val n00 = p00 - 2 * dt * p01 + dt * dt * p11 + ACCEL_NOISE * ACCEL_NOISE * dt
        val n01 = p01 - dt * p11
        val n11 = p11 + BIAS_WALK * BIAS_WALK * dt
        p00 = n00
        p01 = n01
        p11 = n11
    }

    /** Scalar speed measurement with gating; returns whether it was accepted. */
    private fun update(z: Double, r: Double, sensor: Sensor, tNs: Long): Boolean {
        val y = z - v
        val s = p00 + r
        if (y * y > GATE_SIGMA * GATE_SIGMA * s) {
            rejected[sensor] = (rejected[sensor] ?: 0) + 1
            return false
        }
        val k0 = p00 / s
        val k1 = p01 / s
        v += k0 * y
        b += k1 * y
        val n00 = (1 - k0) * p00
        val n01 = (1 - k0) * p01
        val n11 = p11 - k1 * p01
        p00 = n00
        p01 = n01
        p11 = n11
        initialized = true
        markUsed(sensor, tNs)
        return true
    }

    private fun markUsed(sensor: Sensor, tNs: Long) {
        lastUsedNs[sensor] = tNs
    }

    private fun flipAxis() {
        axis[0] = -axis[0]
        axis[1] = -axis[1]
        axis[2] = -axis[2]
        v = -v
        b = -b
        p01 = p01
        sinceStopAlong = -sinceStopAlong
        alongSinceGnss = -alongSinceGnss
    }

    private fun rotateWorldFixed(vec: DoubleArray, dt: Double) {
        val cx = omega[1] * vec[2] - omega[2] * vec[1]
        val cy = omega[2] * vec[0] - omega[0] * vec[2]
        val cz = omega[0] * vec[1] - omega[1] * vec[0]
        vec[0] -= cx * dt
        vec[1] -= cy * dt
        vec[2] -= cz * dt
    }

    /** C' = (I - W dt) C (I - W dt)^T, to first order C - (W C - C W) dt, with W v = omega x v. */
    private fun rotateCov(dt: Double) {
        val w = doubleArrayOf(0.0, -omega[2], omega[1], omega[2], 0.0, -omega[0], -omega[1], omega[0], 0.0)
        val next = DoubleArray(9)
        for (i in 0 until 3) for (j in 0 until 3) {
            var wc = 0.0
            var cw = 0.0
            for (k in 0 until 3) {
                wc += w[i * 3 + k] * cov[k * 3 + j]
                cw += cov[i * 3 + k] * w[k * 3 + j]
            }
            next[i * 3 + j] = cov[i * 3 + j] - (wc - cw) * dt
        }
        next.copyInto(cov)
    }

    /** Exponentially weighted covariance of horizontal acceleration, and its principal axis by power iteration. */
    private fun updateAxis(hx: Double, hy: Double, hz: Double, dt: Double) {
        val k = 1 - exp(-dt / AXIS_TAU_S)
        val h = doubleArrayOf(hx, hy, hz)
        for (i in 0 until 3) for (j in 0 until 3) cov[i * 3 + j] += k * (h[i] * h[j] - cov[i * 3 + j])
        var x = axis[0]
        var y = axis[1]
        var z = axis[2]
        repeat(3) {
            val nx = cov[0] * x + cov[1] * y + cov[2] * z
            val ny = cov[3] * x + cov[4] * y + cov[5] * z
            val nz = cov[6] * x + cov[7] * y + cov[8] * z
            val n = sqrt(nx * nx + ny * ny + nz * nz)
            if (n < 1e-12) return
            x = nx / n
            y = ny / n
            z = nz / n
        }
        // Keep the sign continuous with the previous axis, so a known sign stays known.
        if (x * axis[0] + y * axis[1] + z * axis[2] < 0) {
            x = -x
            y = -y
            z = -z
        }
        axis[0] = x
        axis[1] = y
        axis[2] = z
        if (cov[0] + cov[4] + cov[8] > AXIS_MIN_VARIANCE) axisKnown = true
    }

    private fun distanceM(a: Fix, b: Fix): Double {
        val r = 6_371_008.8
        val p1 = a.lat * PI / 180
        val p2 = b.lat * PI / 180
        val dp = p2 - p1
        val dl = (b.lon - a.lon) * PI / 180
        val h = sin(dp / 2) * sin(dp / 2) + cos(p1) * cos(p2) * sin(dl / 2) * sin(dl / 2)
        return 2 * r * asin(sqrt(minOf(1.0, h)))
    }

    companion object {
        private const val MAX_DT_S = 0.1
        private const val STILL_WINDOW_S = 0.5
        private const val STILL_STD_MPS2 = 0.04
        private const val STILL_S = 2.0
        private const val GRAVITY_TAU_STILL_S = 0.5
        private const val GRAVITY_TAU_MOVING_S = 600.0
        private const val AXIS_TAU_S = 30.0
        private const val AXIS_MIN_VARIANCE = 0.002
        private const val DEPARTURE_EVIDENCE_MPS = 1.0
        private const val SIGN_EVIDENCE = 2.0

        /** Process noise: unmodeled along-axis acceleration (m/s^2 per sqrt(s)), and bias random walk. */
        private const val ACCEL_NOISE = 0.3
        private const val BIAS_WALK = 0.005
        private const val BIAS_SIGMA0 = 0.1
        private const val STOP_SIGMA_MPS = 0.05
        private const val DEFAULT_DOPPLER_SIGMA_MPS = 0.5
        private const val MIN_DOPPLER_SIGMA_MPS = 0.1
        private const val POSITION_MAX_DT_S = 5.0
        private const val POSITION_MAX_SIGMA_MPS = 3.0
        private const val GATE_SIGMA = 3.0
        private const val GNSS_REINIT_REJECTS = 5
        private const val RECENT_S = 3.0
    }
}
