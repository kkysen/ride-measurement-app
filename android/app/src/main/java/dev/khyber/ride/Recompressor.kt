package dev.khyber.ride

import com.github.luben.zstd.Zstd
import com.github.luben.zstd.ZstdException
import com.github.luben.zstd.ZstdInputStream
import com.google.protobuf.CodedInputStream
import com.github.luben.zstd.ZstdOutputStream
import java.io.File
import java.io.FileOutputStream
import java.io.FilterOutputStream
import java.io.IOException
import java.io.InputStream
import java.security.MessageDigest

/**
 * Post-ride recompression: rewrites each stream of a finished session as one zstd frame at a high level.
 *
 * The decoded content (the sequence of length-delimited chunks) is unchanged, so readers don't care.
 * A stream is replaced only after its new file is synced and fully decoded back to the same bytes;
 * any failure, including an original that doesn't decode cleanly (e.g. a crash-truncated tail), keeps the original.
 */
object Recompressor {
    const val LEVEL = 19
    const val MARKER = ".recompressed"
    private const val MAX_STREAM_BYTES = 512L shl 20
    private const val TMP_SUFFIX = ".recompress-tmp"

    sealed interface Result {
        data class Replaced(val before: Long, val after: Long) : Result
        data class KeptOriginal(val reason: String) : Result
    }

    /** Recompresses every stream of [session]; writes [MARKER] once all were handled. */
    fun recompressSession(session: File, level: Int = LEVEL): Map<String, Result> {
        val results = session.listFiles().orEmpty()
            .filter { it.isFile && it.name.endsWith(".$STREAM_EXTENSION") }
            .sortedBy { it.name }
            .associate { it.name to recompressStream(it, level) }
        File(session, MARKER).writeText(
            results.entries.joinToString("\n", postfix = "\n") { (name, r) -> "$name: $r" },
        )
        return results
    }

    /** [corruptForTest] lets tests damage the new file before verification, to check the original survives. */
    fun recompressStream(file: File, level: Int = LEVEL, corruptForTest: ((File) -> Unit)? = null): Result {
        val tmp = File(file.parentFile, file.name + TMP_SUFFIX)
        tmp.delete()
        try {
            if (file.length() > MAX_STREAM_BYTES) return Result.KeptOriginal("too large to check in memory")
            try {
                checkCompleteFrames(file.readBytes())
                file.inputStream().use { checkWholeMessages(ZstdInputStream(it)) }
            } catch (e: IOException) {
                return Result.KeptOriginal("original doesn't decode cleanly: $e")
            }
            val originalDigest = try {
                FileOutputStream(tmp).use { out ->
                    // Closing the zstd stream finishes the frame; keep `out` open so it can be synced afterwards.
                    val unclosable = object : FilterOutputStream(out) {
                        override fun write(b: ByteArray, off: Int, len: Int) = out.write(b, off, len)
                        override fun close() = flush()
                    }
                    val digest = ZstdOutputStream(unclosable, level).use { zout ->
                        file.inputStream().use { digestOf(ZstdInputStream(it), copyTo = zout) }
                    }
                    out.fd.sync()
                    digest
                }
            } catch (e: IOException) {
                return Result.KeptOriginal("writing the new file failed: $e")
            }
            corruptForTest?.invoke(tmp)
            val newDigest = try {
                tmp.inputStream().use { digestOf(ZstdInputStream(it)) }
            } catch (e: IOException) {
                return Result.KeptOriginal("new file doesn't decode: $e")
            }
            if (!newDigest.contentEquals(originalDigest)) return Result.KeptOriginal("decoded content differs")
            val before = file.length()
            val after = tmp.length()
            if (after >= before) return Result.KeptOriginal("not smaller ($after >= $before bytes)")
            // `rename` replaces the original atomically: either the old or the verified new file exists, never neither.
            if (!tmp.renameTo(file)) return Result.KeptOriginal("rename failed")
            return Result.Replaced(before, after)
        } finally {
            tmp.delete()
        }
    }

    /** Throws unless [bytes] is entirely complete zstd frames; the stream decoder alone accepts a cut-off last frame. */
    private fun checkCompleteFrames(bytes: ByteArray) {
        var offset = 0
        while (offset < bytes.size) {
            val size = try {
                Zstd.findFrameCompressedSize(bytes, offset)
            } catch (e: ZstdException) {
                throw IOException("incomplete zstd frame at byte $offset of ${bytes.size}", e)
            }
            if (Zstd.isError(size) || size <= 0 || offset + size > bytes.size) {
                throw IOException("incomplete zstd frame at byte $offset of ${bytes.size}")
            }
            offset += size.toInt()
        }
    }

    /** Throws unless [input] decodes to a whole number of length-delimited messages, as after a crash mid-frame. */
    private fun checkWholeMessages(input: InputStream) {
        val coded = CodedInputStream.newInstance(input)
        coded.setSizeLimit(Int.MAX_VALUE)
        while (!coded.isAtEnd) {
            val length = coded.readRawVarint32()
            coded.skipRawBytes(length)
            coded.resetSizeCounter()
        }
    }

    /** SHA-256 and length of everything [input] yields, optionally copying it to [copyTo]. */
    private fun digestOf(input: InputStream, copyTo: java.io.OutputStream? = null): ByteArray {
        val sha = MessageDigest.getInstance("SHA-256")
        val buf = ByteArray(1 shl 16)
        var length = 0L
        input.use {
            while (true) {
                val n = it.read(buf)
                if (n < 0) break
                sha.update(buf, 0, n)
                copyTo?.write(buf, 0, n)
                length += n
            }
        }
        sha.update(java.nio.ByteBuffer.allocate(8).putLong(length).array())
        return sha.digest()
    }
}
