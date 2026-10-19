package dev.khyber.ride

import android.os.Handler
import android.os.HandlerThread
import android.os.SystemClock
import dev.khyber.ride.proto.Record
import dev.khyber.ride.proto.RecordChunk
import dev.khyber.ride.proto.StreamInfo
import dev.khyber.ride.proto.StreamKind
import java.io.File
import uniffi.ride_ffi.StreamWriter

/** A record stream: thread-safe [add], written as one chunk every [FLUSH_MS] on its own thread. */
class RecordStreamRecorder(val name: String, dir: File) {
    private val thread = HandlerThread("records-$name").apply { start() }
    val handler = Handler(thread.looper)
    private val writer = StreamWriter.create(dir.path, name)
    private val pending = ArrayList<Record>()

    @Volatile
    var recordCount = 0L
        private set

    val streamInfo: StreamInfo
        get() = StreamInfo.newBuilder().setName(name).setKind(StreamKind.STREAM_KIND_RECORDS).build()

    private val flushTask = object : Runnable {
        override fun run() {
            Health.guard("write $name") { flush() }
            handler.postDelayed(this, FLUSH_MS)
        }
    }

    init {
        handler.postDelayed(flushTask, FLUSH_MS)
    }

    /**
     * Stamps the record with the receive time now, on the caller's thread, then queues it.
     * If building it throws (e.g. an Android getter fails), that's reported to [Health] and the record skipped.
     */
    fun add(build: Record.Builder.() -> Unit) {
        val receivedNs = SystemClock.elapsedRealtimeNanos()
        val record = Health.guard("records $name") {
            Record.newBuilder().setReceivedElapsedRealtimeNs(receivedNs).apply(build).build()
        } ?: return
        handler.post {
            pending.add(record)
            recordCount++
        }
    }

    fun stop() {
        handler.removeCallbacks(flushTask)
        handler.post {
            Health.guard("write $name") { flush() }
            Health.guard("close $name") { writer.finish() }
            writer.close()
        }
        thread.quitSafely()
        thread.join()
    }

    /** Hands the pending records to the writer, which keeps them and retries if the write fails. */
    private fun flush() {
        if (pending.isEmpty()) return
        val chunk = RecordChunk.newBuilder()
            .addAllRecords(pending)
            .setWrittenElapsedRealtimeNs(SystemClock.elapsedRealtimeNanos())
            .build()
            .toByteArray()
        pending.clear()
        writer.appendEncoded(chunk)
    }

    companion object {
        const val FLUSH_MS = 2_000L
    }
}
