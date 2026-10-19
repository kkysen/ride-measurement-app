package dev.khyber.ride

import dev.khyber.ride.proto.SessionInfo
import dev.khyber.ride.proto.StreamInfo
import java.io.File
import org.junit.Assert.assertTrue
import org.junit.Test
import uniffi.ride_ffi.StreamWriter
import uniffi.ride_ffi.formatVersion
import uniffi.ride_ffi.recompressSession
import uniffi.ride_ffi.writeSessionInfo
import uniffi.ride_ffi.recompressedMarker

/**
 * Writes a session through the app's Rust bindings, exactly as the recorders do, for the Rust and Python readers
 * to check bit-exactly (`crates/session/tests/kotlin_fixture.rs`): values cross from Kotlin as raw bits,
 * and nothing on the way may alter them.
 */
class FixtureTest {
    @Test
    fun writeFixture() {
        val dir = File(System.getProperty("ride.fixtureDir") ?: "build/fixtures/kotlin-session")
        dir.deleteRecursively()
        dir.mkdirs()
        writeSessionInfo(
            dir.path,
            SessionInfo.newBuilder()
                .setFormatVersion(formatVersion().toInt())
                .setSessionId("kotlin-fixture")
                .addStreams(StreamInfo.newBuilder().setName("accel"))
                .build()
                .toByteArray(),
        )
        StreamWriter.create(dir.path, "accel").use { writer ->
            for (c in 0 until CHUNKS) {
                // As `SensorStreamRecorder` does: one call per chunk, values as raw bits.
                val samples = (0 until SAMPLES_PER_CHUNK).map { c * SAMPLES_PER_CHUNK + it }
                writer.appendSensorSamples(
                    samples.map { fixtureTimestamp(it) },
                    3u,
                    samples.flatMap { s -> (0 until 3).map { k -> fixtureValue(s * 3 + k).toRawBits().toUInt() } },
                    samples.map { it % 4 },
                    c.toLong(),
                )
            }
            writer.finish()
        }
    }

    /**
     * The same session after post-ride recompression, through the bindings the app uses,
     * for the Rust and Python readers to check (`kotlin_fixture.rs`, `test_ride.py`).
     */
    @Test
    fun writeRecompressedFixture() {
        val source = File(System.getProperty("ride.fixtureDir") ?: "build/fixtures/kotlin-session")
        writeFixture()
        val dir = File(source.parentFile, "kotlin-session-recompressed")
        dir.deleteRecursively()
        source.copyRecursively(dir)
        val results = recompressSession(dir.path)
        assertTrue(results.toString(), results.isNotEmpty() && results.all { it.replaced })
        assertTrue(File(dir, recompressedMarker()).exists())
    }

    companion object {
        const val CHUNKS = 3
        const val SAMPLES_PER_CHUNK = 100

        // Keep in sync with `crates/session/tests/kotlin_fixture.rs`.
        private val TRICKY_BITS = intArrayOf(
            0x00000000, 0x80000000.toInt(), 0x00800000, 0x00000001, 0x7f7fffff,
            0x7f800000, 0xff800000.toInt(), 0x7fc01234, 0x411ce80a, 0xb4a5b4b2.toInt(),
        )

        fun fixtureValue(i: Int): Float = java.lang.Float.intBitsToFloat(TRICKY_BITS[i % TRICKY_BITS.size] xor i)
        fun fixtureTimestamp(sample: Int): Long = 1_000_000_000L + sample * 2_500_000L + sample % 3
    }
}
