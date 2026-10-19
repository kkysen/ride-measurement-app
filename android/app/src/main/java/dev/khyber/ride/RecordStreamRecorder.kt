package dev.khyber.ride

import android.os.Handler
import android.os.HandlerThread
import android.os.SystemClock
import dev.khyber.ride.proto.Record
import dev.khyber.ride.proto.RecordChunk
import dev.khyber.ride.proto.StreamInfo
import dev.khyber.ride.proto.StreamKind
import java.io.File

/** A record stream: thread-safe [add], written as one chunk every [FLUSH_MS] on its own thread. */
class RecordStreamRecorder(val name: String, dir: File) {
    private val thread = HandlerThread("records-$name").apply { start() }
    val handler = Handler(thread.looper)
    private val writer = StreamWriter(dir, name)
    private val pending = ArrayList<Record>()

    @Volatile
    var recordCount = 0L
        private set

    val streamInfo: StreamInfo
        get() = StreamInfo.newBuilder().setName(name).setKind(StreamKind.STREAM_KIND_RECORDS).build()

    private val flushTask = object : Runnable {
        override fun run() {
            flush()
            handler.postDelayed(this, FLUSH_MS)
        }
    }

    init {
        handler.postDelayed(flushTask, FLUSH_MS)
    }

    /** Stamps the record with the receive time now, on the caller's thread, then queues it. */
    fun add(build: Record.Builder.() -> Unit) {
        val record = Record.newBuilder().setReceivedElapsedRealtimeNs(SystemClock.elapsedRealtimeNanos()).apply(build).build()
        handler.post {
            pending.add(record)
            recordCount++
        }
    }

    fun stop() {
        handler.removeCallbacks(flushTask)
        handler.post {
            flush()
            writer.close()
        }
        thread.quitSafely()
        thread.join()
    }

    private fun flush() {
        if (pending.isEmpty()) return
        writer.append(
            RecordChunk.newBuilder()
                .addAllRecords(pending)
                .setWrittenElapsedRealtimeNs(SystemClock.elapsedRealtimeNanos())
                .build(),
        )
        pending.clear()
    }

    companion object {
        const val FLUSH_MS = 2_000L
    }
}
