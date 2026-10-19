package dev.khyber.ride

import dev.khyber.ride.proto.SessionInfo
import dev.khyber.ride.proto.StreamInfo
import java.io.File
import org.junit.Test

/**
 * Writes a session with the Kotlin writer for the Rust reader to check bit-exactly
 * (`crates/session/tests/kotlin_fixture.rs`), so both sides of the format agree.
 */
class FixtureTest {
    @Test
    fun writeFixture() {
        val dir = File(System.getProperty("ride.fixtureDir") ?: "build/fixtures/kotlin-session")
        dir.deleteRecursively()
        SessionWriter.writeSessionInfo(
            dir,
            SessionInfo.newBuilder()
                .setFormatVersion(FORMAT_VERSION)
                .setSessionId("kotlin-fixture")
                .addStreams(StreamInfo.newBuilder().setName("accel"))
                .build(),
        )
        StreamWriter(dir, "accel").use { writer ->
            for (c in 0 until CHUNKS) {
                val builder = ChunkBuilder(3)
                for (i in 0 until SAMPLES_PER_CHUNK) {
                    val sample = c * SAMPLES_PER_CHUNK + i
                    val values = FloatArray(3) { k -> fixtureValue(sample * 3 + k) }
                    builder.add(fixtureTimestamp(sample), values, sample % 4)
                }
                writer.append(builder.build(c.toLong()))
            }
        }
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
