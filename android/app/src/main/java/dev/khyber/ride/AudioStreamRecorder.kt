package dev.khyber.ride

import android.annotation.SuppressLint
import android.content.Context
import android.media.AudioFormat
import android.media.AudioManager
import android.media.AudioRecord
import android.media.AudioTimestamp
import android.media.MediaRecorder
import android.os.SystemClock
import android.util.Log
import com.google.protobuf.ByteString
import dev.khyber.ride.proto.AudioChunk
import dev.khyber.ride.proto.AudioInfo
import dev.khyber.ride.proto.AudioTimestamp as PbAudioTimestamp
import dev.khyber.ride.proto.StreamInfo
import dev.khyber.ride.proto.StreamKind
import java.io.ByteArrayOutputStream
import java.io.File
import uniffi.ride_ffi.StreamWriter

/**
 * Lossless mono 16-bit PCM from the microphone into the `audio` stream, one chunk per [CHUNK_FRAMES],
 * with `AudioRecord` boot-time timestamps so it lines up with the sensors.
 * Uses the `UNPROCESSED` source when the phone supports it, so no gain control or noise suppression alters it.
 */
class AudioStreamRecorder(context: Context, dir: File) {
    private val unprocessed = context.getSystemService(AudioManager::class.java)
        .getProperty(AudioManager.PROPERTY_SUPPORT_AUDIO_SOURCE_UNPROCESSED) == "true"
    private val source = if (unprocessed) MediaRecorder.AudioSource.UNPROCESSED else MediaRecorder.AudioSource.VOICE_RECOGNITION
    private val writer = StreamWriter.create(dir.path, NAME)
    private var record: AudioRecord? = null
    private var thread: Thread? = null

    @Volatile
    private var running = false

    @Volatile
    var frameCount = 0L
        private set

    val streamInfo: StreamInfo
        get() = StreamInfo.newBuilder()
            .setName(NAME)
            .setKind(StreamKind.STREAM_KIND_AUDIO)
            .setAudio(
                AudioInfo.newBuilder()
                    .setSampleRateHz(SAMPLE_RATE_HZ)
                    .setChannels(1)
                    .setAudioSource(source)
                    .setAudioSourceName(if (unprocessed) "UNPROCESSED" else "VOICE_RECOGNITION"),
            )
            .build()

    @SuppressLint("MissingPermission") // Only started after `RECORD_AUDIO` is checked.
    fun start() {
        val minBuffer = AudioRecord.getMinBufferSize(SAMPLE_RATE_HZ, AudioFormat.CHANNEL_IN_MONO, AudioFormat.ENCODING_PCM_16BIT)
        val r = AudioRecord(source, SAMPLE_RATE_HZ, AudioFormat.CHANNEL_IN_MONO, AudioFormat.ENCODING_PCM_16BIT, maxOf(minBuffer, READ_FRAMES * 2 * 8))
        record = r
        running = true
        r.startRecording()
        thread = Thread({ Health.guard("audio") { loop(r) } }, "audio").apply { start() }
    }

    fun stop() {
        running = false
        thread?.join()
        record?.run {
            stop()
            release()
        }
        Health.guard("close audio") { writer.finish() }
        writer.close()
    }

    private fun loop(r: AudioRecord) {
        val buf = ByteArray(READ_FRAMES * 2)
        val pcm = ByteArrayOutputStream(CHUNK_FRAMES * 2)
        val timestamps = ArrayList<PbAudioTimestamp>()
        val ts = AudioTimestamp()
        var chunkFirstFrame = 0L
        // Hands the chunk to the writer, which keeps it and retries if the write fails, so it's cleared either way.
        fun flush() {
            if (pcm.size() == 0) return
            val chunk = Health.guard("audio chunk") {
                AudioChunk.newBuilder()
                    .setFirstFrame(chunkFirstFrame)
                    .setPcmS16Le(ByteString.copyFrom(pcm.toByteArray()))
                    .addAllTimestamps(timestamps)
                    .setWrittenElapsedRealtimeNs(SystemClock.elapsedRealtimeNanos())
                    .build()
                    .toByteArray()
            }
            pcm.reset()
            timestamps.clear()
            chunkFirstFrame = frameCount
            if (chunk != null) Health.guard("write audio") { writer.appendEncoded(chunk) }
        }
        while (running) {
            val n = r.read(buf, 0, buf.size)
            if (n < 0) {
                Health.report("audio", java.io.IOException("AudioRecord.read returned $n"))
                break
            }
            pcm.write(buf, 0, n)
            frameCount += n / 2
            if (r.getTimestamp(ts, AudioTimestamp.TIMEBASE_BOOTTIME) == AudioRecord.SUCCESS) {
                timestamps.add(PbAudioTimestamp.newBuilder().setFramePosition(ts.framePosition).setBoottimeNs(ts.nanoTime).build())
            }
            if (pcm.size() >= CHUNK_FRAMES * 2) flush()
        }
        flush()
    }

    companion object {
        const val NAME = "audio"
        const val SAMPLE_RATE_HZ = 16_000
        private const val READ_FRAMES = SAMPLE_RATE_HZ / 10
        private const val CHUNK_FRAMES = SAMPLE_RATE_HZ * 2

        private const val PREFS = "audio"
        private const val ENABLED = "enabled"

        fun enabled(context: Context) = context.getSharedPreferences(PREFS, Context.MODE_PRIVATE).getBoolean(ENABLED, false)

        fun setEnabled(context: Context, enabled: Boolean) {
            context.getSharedPreferences(PREFS, Context.MODE_PRIVATE).edit().putBoolean(ENABLED, enabled).apply()
        }
    }
}
