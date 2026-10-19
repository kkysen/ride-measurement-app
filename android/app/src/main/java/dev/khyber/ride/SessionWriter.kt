package dev.khyber.ride

import com.github.luben.zstd.ZstdCompressCtx
import com.google.protobuf.MessageLite
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

    // Frame checksums let readers tell a corrupted frame from valid data.
    private val ctx = ZstdCompressCtx().setLevel(LIVE_ZSTD_LEVEL).setChecksum(true)

    fun append(chunk: MessageLite) {
        val bytes = ByteArrayOutputStream().also { chunk.writeDelimitedTo(it) }.toByteArray()
        out.write(ctx.compress(bytes))
        out.fd.sync()
    }

    override fun close() {
        out.close()
        ctx.close()
    }
}

/**
 * Accumulates samples and builds delta-encoded chunks with exact `f32` bits.
 * Every value Android delivers is kept: a chunk's `values_per_sample` is the first sample's `values.size`,
 * and a sample of a different size needs a new chunk (see [fits]).
 */
class ChunkBuilder {
    private val timestamps = ArrayList<Long>()
    private val valueBits = ArrayList<Int>()
    private val accuracy = ArrayList<Int>()
    private var valuesPerSample = 0

    val size get() = timestamps.size

    /** Whether a sample with [valueCount] values can go in this chunk. */
    fun fits(valueCount: Int) = size == 0 || valueCount == valuesPerSample

    fun add(timestampNs: Long, values: FloatArray, accuracy: Int) {
        require(fits(values.size)) { "sample has ${values.size} values, chunk has $valuesPerSample" }
        if (size == 0) valuesPerSample = values.size
        timestamps.add(timestampNs)
        for (i in values.indices) {
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
