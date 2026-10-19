package dev.khyber.ride

import com.github.luben.zstd.Zstd
import dev.khyber.ride.proto.SensorChunk
import dev.khyber.ride.proto.SessionInfo
import java.io.ByteArrayOutputStream
import java.io.File
import java.io.FileOutputStream

const val FORMAT_VERSION = 1
const val SESSION_INFO_FILE = "session.pb"
const val STREAM_EXTENSION = "zst"

/** Level used while recording; cheap, since post-ride recompression squeezes further. */
const val LIVE_ZSTD_LEVEL = 3

/** Mirrors `crates/session/src/write.rs`; the Rust reader is the reference for the format. */
object SessionWriter {
    fun writeSessionInfo(dir: File, info: SessionInfo) {
        dir.mkdirs()
        FileOutputStream(File(dir, SESSION_INFO_FILE)).use { out ->
            info.writeDelimitedTo(out)
            out.fd.sync()
        }
    }
}

/** Appends chunks to `<dir>/<name>.zst`, one independent zstd frame per chunk, synced after each. */
class StreamWriter(dir: File, name: String) : AutoCloseable {
    val file = File(dir, "$name.$STREAM_EXTENSION")
    private val out: FileOutputStream

    init {
        check(file.createNewFile()) { "$file already exists" }
        out = FileOutputStream(file, true)
    }

    fun append(chunk: SensorChunk) {
        val bytes = ByteArrayOutputStream().also { chunk.writeDelimitedTo(it) }.toByteArray()
        out.write(Zstd.compress(bytes, LIVE_ZSTD_LEVEL))
        out.fd.sync()
    }

    override fun close() = out.close()
}

/** Accumulates samples and builds delta-encoded chunks with exact `f32` bits. */
class ChunkBuilder(private val valuesPerSample: Int) {
    private val timestamps = ArrayList<Long>()
    private val valueBits = ArrayList<Int>()
    private val accuracy = ArrayList<Int>()

    val size get() = timestamps.size

    fun add(timestampNs: Long, values: FloatArray, accuracy: Int) {
        require(values.size >= valuesPerSample)
        timestamps.add(timestampNs)
        for (i in 0 until valuesPerSample) {
            // `floatToRawIntBits` keeps NaN payloads, unlike `floatToIntBits`.
            valueBits.add(java.lang.Float.floatToRawIntBits(values[i]))
        }
        this.accuracy.add(accuracy)
    }

    fun build(writtenElapsedRealtimeNs: Long): SensorChunk {
        var prev = 0L
        val chunk = SensorChunk.newBuilder()
            .setValuesPerSample(valuesPerSample)
            .addAllTimestampDeltasNs(timestamps.map { t -> (t - prev).also { prev = t } })
            .addAllValueBits(valueBits)
            .addAllAccuracy(accuracy)
            .setWrittenElapsedRealtimeNs(writtenElapsedRealtimeNs)
            .build()
        timestamps.clear()
        valueBits.clear()
        accuracy.clear()
        return chunk
    }
}
