package dev.khyber.ride

import com.github.luben.zstd.ZstdInputStream
import dev.khyber.ride.proto.SessionInfo
import dev.khyber.ride.proto.StreamInfo
import java.io.File
import java.io.RandomAccessFile
import java.nio.file.Files
import org.junit.Assert.assertArrayEquals
import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test

class RecompressorTest {
    private fun tempDir(): File = Files.createTempDirectory("recompress").toFile().apply { deleteOnExit() }

    /** A stream of [chunks] chunks in the live format: one zstd frame each. */
    private fun writeStream(dir: File, name: String, chunks: Int): File {
        StreamWriter(dir, name).use { writer ->
            for (c in 0 until chunks) {
                val builder = ChunkBuilder()
                for (i in 0 until 200) {
                    val sample = c * 200 + i
                    builder.add(1_000_000_000L + sample * 2_500_000L, FloatArray(3) { k -> FixtureTest.fixtureValue(sample * 3 + k) }, 3)
                }
                writer.append(builder.build(c.toLong()))
            }
        }
        return File(dir, "$name.$STREAM_EXTENSION")
    }

    private fun decoded(file: File): ByteArray = file.inputStream().use { ZstdInputStream(it).readBytes() }

    @Test
    fun replacesWithSmallerFileWithIdenticalContent() {
        val dir = tempDir()
        val file = writeStream(dir, "accel", 50)
        val before = decoded(file)
        val result = Recompressor.recompressStream(file)
        assertTrue(result.toString(), result is Recompressor.Result.Replaced)
        assertArrayEquals(before, decoded(file))
        assertTrue((result as Recompressor.Result.Replaced).after < result.before)
        assertEquals(listOf("accel.zst"), dir.list()!!.toList())
    }

    @Test
    fun keepsOriginalWhenVerificationFails() {
        val dir = tempDir()
        val file = writeStream(dir, "accel", 50)
        val originalBytes = file.readBytes()
        val result = Recompressor.recompressStream(file) { tmp ->
            RandomAccessFile(tmp, "rw").use { raf ->
                raf.seek(raf.length() / 2)
                val b = raf.read()
                raf.seek(raf.length() / 2)
                raf.write(b xor 0xff)
            }
        }
        assertTrue(result.toString(), result is Recompressor.Result.KeptOriginal)
        assertArrayEquals(originalBytes, file.readBytes())
        assertEquals(listOf("accel.zst"), dir.list()!!.toList())
    }

    @Test
    fun keepsCrashTruncatedOriginal() {
        val dir = tempDir()
        val file = writeStream(dir, "accel", 5)
        RandomAccessFile(file, "rw").use { it.setLength(it.length() - 10) }
        val originalBytes = file.readBytes()
        val result = Recompressor.recompressStream(file)
        assertTrue(result.toString(), result is Recompressor.Result.KeptOriginal)
        assertArrayEquals(originalBytes, file.readBytes())
    }

    /** A recompressed session for the Rust and Python readers to check (`kotlin_fixture.rs`, `test_ride.py`). */
    @Test
    fun writeRecompressedFixture() {
        val source = File(System.getProperty("ride.fixtureDir") ?: "build/fixtures/kotlin-session")
        FixtureTest().writeFixture()
        val dir = File(source.parentFile, "kotlin-session-recompressed")
        dir.deleteRecursively()
        source.copyRecursively(dir)
        val results = Recompressor.recompressSession(dir)
        assertTrue(results.toString(), results.values.all { it is Recompressor.Result.Replaced })
        assertTrue(File(dir, Recompressor.MARKER).exists())
    }

    @Test
    fun sessionInfoIsUntouched() {
        val dir = tempDir()
        SessionWriter.writeSessionInfo(
            dir,
            SessionInfo.newBuilder().setFormatVersion(FORMAT_VERSION).addStreams(StreamInfo.newBuilder().setName("accel")).build(),
        )
        writeStream(dir, "accel", 3)
        val info = File(dir, SESSION_INFO_FILE).readBytes()
        Recompressor.recompressSession(dir)
        assertArrayEquals(info, File(dir, SESSION_INFO_FILE).readBytes())
    }
}
