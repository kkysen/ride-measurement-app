package dev.khyber.ride

import org.junit.Assert.assertTrue
import org.junit.Test
import uniffi.ride_ffi.Fusion
import uniffi.ride_ffi.StreamWriter
import java.nio.file.Files

/**
 * What the recording path's calls into Rust cost per sample, on this machine's JVM as a rough guide:
 * each call costs tens of microseconds, so samples cross in batches (a chunk to the writer, 64 to live speed).
 */
class BindingCostTest {
    @Test
    fun perSampleCostIsSmallWhenBatched() {
        val dir = Files.createTempDirectory("cost").toFile().apply { deleteOnExit() }
        val chunkSamples = 800
        val liveBatch = 64
        val bits = List(chunkSamples * 6) { 0x3f800000u }
        val floats = List(liveBatch * 6) { 1f }
        StreamWriter.create(dir.path, "accel").use { w ->
            Fusion().use { f ->
                fun chunk(c: Int) {
                    val t0 = c * chunkSamples * 2_500_000L
                    for (b in 0 until chunkSamples / liveBatch) {
                        f.onSensorSamples(35, List(liveBatch) { t0 + (b * liveBatch + it) * 2_500_000L }, 6u, floats)
                    }
                    w.appendSensorSamples(List(chunkSamples) { t0 + it * 2_500_000L }, 6u, bits, List(chunkSamples) { 3 }, 0)
                }
                repeat(5) { chunk(it) } // warm up
                val chunks = 50
                val start = System.nanoTime()
                for (c in 5 until 5 + chunks) chunk(c)
                val perSampleUs = (System.nanoTime() - start) / 1e3 / (chunks * chunkSamples)
                println("per sample, batched: %.2f us".format(perSampleUs))
                assertTrue("per sample $perSampleUs us", perSampleUs < 10)
                w.finish()
            }
        }
    }

    @Test
    fun smoothingAFullWindow() {
        Fusion().use { f ->
            val batch = List(400 * 6) { if (it % 6 == 2) 9.8f else 0f }
            // 10 minutes of accelerometer and gyro at 400 Hz, and a fix a second: smoothing re-runs every second.
            val start = System.nanoTime()
            for (s in 0 until 600) {
                val ts = List(400) { (s * 400L + it) * 2_500_000 }
                f.onSensorSamples(35, ts, 6u, batch)
                f.onSensorSamples(16, ts, 6u, batch)
                f.onFix(ts[0], 40.0, -74.0, 10.0, 5.0, 10.0, 0.3)
            }
            val ms = (System.nanoTime() - start) / 1e6
            val smoothed = f.smoothed()
            println("600 s at 400 Hz, smoothed every second: %.0f ms, %.1f ms per second of input, %d points".format(ms, ms / 600, smoothed.size))
            assertTrue(smoothed.size >= 600)
        }
    }
}
