package dev.khyber.ride

import android.annotation.SuppressLint
import android.content.Context
import android.media.AudioDeviceInfo
import android.media.AudioFormat
import android.media.AudioManager
import android.media.AudioRecord
import android.media.AudioTimestamp
import android.media.MediaRecorder
import android.media.MicrophoneInfo as AndroidMicrophoneInfo
import android.os.SystemClock
import android.util.Log
import com.google.protobuf.ByteString
import dev.khyber.ride.proto.AudioChunk
import dev.khyber.ride.proto.AudioInfo
import dev.khyber.ride.proto.AudioTimestamp as PbAudioTimestamp
import dev.khyber.ride.proto.Coordinate3F
import dev.khyber.ride.proto.MicrophoneInfo
import dev.khyber.ride.proto.StreamInfo
import dev.khyber.ride.proto.StreamKind
import java.io.ByteArrayOutputStream
import java.io.File
import uniffi.ride_ffi.StreamWriter

/**
 * Lossless 16-bit PCM from every microphone at once (one interleaved channel each) at the input's highest
 * native rate, into the `audio` stream, two seconds per chunk, with `AudioRecord` boot-time timestamps
 * so it lines up with the sensors.
 * Uses the `DEFAULT` source on the default input: on the S25 (Galaxy SM-S947U1) it's the only way both microphones
 * are recorded (`MicProbe`, 2026-11-09); `VOICE_RECOGNITION` and `VOICE_COMMUNICATION` give only the bottom one,
 * asking for a particular microphone gives it twice, `UNPROCESSED` only the bottom one with a strange 2nd channel,
 * and `MIC` and `CAMCORDER` clipped the back one on speech.
 * The microphones, what the input supports, and any format Android refused are all written into the session.
 */
@SuppressLint("MissingPermission") // Only made after `RECORD_AUDIO` is checked.
class AudioStreamRecorder(context: Context, dir: File) {
    private val audioManager = context.getSystemService(AudioManager::class.java)
    private val source = MediaRecorder.AudioSource.DEFAULT
    private val inputs = audioManager.getDevices(AudioManager.GET_DEVICES_INPUTS)
    /** The phone's own microphone, for its rates and channels. */
    private val builtInMic = inputs.firstOrNull { it.type == AudioDeviceInfo.TYPE_BUILTIN_MIC }
    /**
     * Asked for only when a headset or other microphone is connected, so it doesn't take over:
     * asking for the phone's own records one microphone, not both.
     */
    private val preferBuiltIn = inputs.any {
        it.type != AudioDeviceInfo.TYPE_BUILTIN_MIC && it.type != AudioDeviceInfo.TYPE_TELEPHONY &&
            it.type != AudioDeviceInfo.TYPE_REMOTE_SUBMIX && it.type != AudioDeviceInfo.TYPE_FM_TUNER
    }
    private val microphones = Health.guard("microphones") { audioManager.microphones } ?: emptyList()
    private val refused = ArrayList<String>()
    private val record: AudioRecord = open()
    val sampleRateHz: Int = record.sampleRate
    val channels: Int = record.channelCount
    private val readFrames = sampleRateHz / 10
    private val chunkFrames = sampleRateHz * 2
    private val writer = StreamWriter.create(dir.path, NAME)
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
                    .setSampleRateHz(sampleRateHz)
                    .setChannels(channels)
                    .setAudioSource(source)
                    .setAudioSourceName("DEFAULT")
                    .setInputDevice(builtInMic?.let { "${it.productName} ${it.address}".trim() } ?: "")
                    .addAllMicrophones(microphones.map(::microphoneInfo))
                    .addAllDeviceSampleRates(builtInMic?.sampleRates?.toList() ?: emptyList())
                    .addAllDeviceChannelCounts(builtInMic?.channelCounts?.toList() ?: emptyList())
                    .addAllDeviceChannelIndexMasks(builtInMic?.channelIndexMasks?.toList() ?: emptyList())
                    .addAllDeviceChannelMasks(builtInMic?.channelMasks?.toList() ?: emptyList())
                    .setChannelIndexMask(record.format.channelIndexMask)
                    .setEncoding(record.format.encoding)
                    .addAllRefusedFormats(refused),
            )
            .build()

    /**
     * The highest native rate the input lists (48 kHz if it lists none: Android resamples any rate asked for,
     * and more than native is no more data), with a channel per microphone (more are silent or copies on the S25),
     * or as many as the input says it has if Android lists no microphones.
     * Fewer channels only if Android refuses, each refusal kept for the session.
     */
    private fun open(): AudioRecord {
        val rates = builtInMic?.sampleRates?.filter { it > 0 }?.sortedDescending()?.takeIf { it.isNotEmpty() } ?: listOf(48_000)
        val builtIn = microphones.count { it.type == AudioDeviceInfo.TYPE_BUILTIN_MIC }
        val most = if (builtIn > 0) builtIn else maxOf(
            builtInMic?.channelCounts?.maxOrNull() ?: 0,
            builtInMic?.channelIndexMasks?.maxOfOrNull { Integer.bitCount(it) } ?: 0,
            1,
        )
        for (rate in rates) {
            for (n in most downTo 1) {
                val format = AudioFormat.Builder()
                    .setEncoding(AudioFormat.ENCODING_PCM_16BIT)
                    .setSampleRate(rate)
                    .setChannelIndexMask((1 shl n) - 1)
                    .build()
                tryOpen(format, "$rate Hz, ${n} channels by index")?.let { return it }
            }
        }
        val mono = AudioFormat.Builder()
            .setEncoding(AudioFormat.ENCODING_PCM_16BIT)
            .setSampleRate(rates.first())
            .setChannelMask(AudioFormat.CHANNEL_IN_MONO)
            .build()
        return tryOpen(mono, "${rates.first()} Hz mono") ?: throw IllegalStateException("no audio format opened: $refused")
    }

    private fun tryOpen(format: AudioFormat, label: String): AudioRecord? {
        val r = try {
            val frameBytes = 2 * format.channelCount
            val minBuffer = AudioRecord.getMinBufferSize(format.sampleRate, format.channelMask, format.encoding)
            AudioRecord.Builder()
                .setAudioSource(source)
                .setAudioFormat(format)
                .setBufferSizeInBytes(maxOf(minBuffer, format.sampleRate / 10 * frameBytes * 8))
                .build()
        } catch (e: Exception) {
            refused.add("$label: $e")
            return null
        }
        if (r.state != AudioRecord.STATE_INITIALIZED || r.channelCount != format.channelCount || r.sampleRate != format.sampleRate) {
            refused.add("$label: got state ${r.state}, ${r.sampleRate} Hz, ${r.channelCount} channels")
            r.release()
            return null
        }
        if (preferBuiltIn) builtInMic?.let { r.setPreferredDevice(it) }
        return r
    }

    fun start() {
        val r = record
        running = true
        r.startRecording()
        // Shown and written into the session if Android records from something else anyway (e.g. a wired headset).
        r.routedDevice?.takeIf { it.type != AudioDeviceInfo.TYPE_BUILTIN_MIC }?.let {
            Health.report("audio", IllegalStateException("recording from ${it.productName} (type ${it.type}), not the phone's microphone"))
        }
        val builtIn = microphones.count { it.type == AudioDeviceInfo.TYPE_BUILTIN_MIC }
        if (channels < builtIn) {
            Health.report("audio", IllegalStateException("recording $channels of $builtIn microphones: Android refused more ($refused)"))
        }
        thread = Thread({ Health.guard("audio") { loop(r) } }, "audio").apply { start() }
    }

    fun stop() {
        running = false
        thread?.join()
        record.run {
            stop()
            release()
        }
        Health.guard("close audio") { writer.finish() }
        writer.close()
    }

    private fun loop(r: AudioRecord) {
        val frameBytes = 2 * channels
        val buf = ByteArray(readFrames * frameBytes)
        val pcm = ByteArrayOutputStream(chunkFrames * frameBytes)
        val sameness = ChannelSameness(channels, sampleRateHz * SAMENESS_S)
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
            val readFirstFrame = frameCount
            frameCount += n / frameBytes
            sameness.add(buf, n)?.let { Health.report("audio", IllegalStateException(it)) }
            val stamped = r.getTimestamp(ts, AudioTimestamp.TIMEBASE_BOOTTIME) == AudioRecord.SUCCESS
            if (stamped) {
                timestamps.add(PbAudioTimestamp.newBuilder().setFramePosition(ts.framePosition).setBoottimeNs(ts.nanoTime).build())
            }
            // To live speed as read, timed at its last frame, as `crates/fusion/src/replay.rs` replays it.
            Health.guard("live audio") {
                val last = timestamps.lastOrNull()
                val tNs = if (last != null) {
                    last.boottimeNs + (frameCount - last.framePosition) * 1_000_000_000L / sampleRateHz
                } else {
                    SystemClock.elapsedRealtimeNanos()
                }
                LiveSpeed.onAudio(
                    tNs,
                    sampleRateHz,
                    channels,
                    readFirstFrame,
                    buf.copyOf(n),
                    false,
                    if (stamped) listOf(ts.framePosition) else emptyList(),
                    if (stamped) listOf(ts.nanoTime) else emptyList(),
                )
            }
            if (pcm.size() >= chunkFrames * frameBytes) flush()
        }
        flush()
    }

    companion object {
        const val NAME = "audio"
        /** How long the channels are compared at the start, to tell copies of one microphone from several. */
        private const val SAMENESS_S = 10

        private const val PREFS = "audio"
        private const val ENABLED = "enabled"

        fun enabled(context: Context) = context.getSharedPreferences(PREFS, Context.MODE_PRIVATE).getBoolean(ENABLED, false)

        fun setEnabled(context: Context, enabled: Boolean) {
            context.getSharedPreferences(PREFS, Context.MODE_PRIVATE).edit().putBoolean(ENABLED, enabled).apply()
        }
    }
}

private fun microphoneInfo(m: AndroidMicrophoneInfo): MicrophoneInfo {
    fun c(p: AndroidMicrophoneInfo.Coordinate3F) = Coordinate3F.newBuilder().setX(p.x).setY(p.y).setZ(p.z).build()
    return MicrophoneInfo.newBuilder()
        .setDescription(m.description)
        .setId(m.id)
        .setType(m.type)
        .setAddress(m.address)
        .setLocation(m.location)
        .setGroup(m.group)
        .setIndexInTheGroup(m.indexInTheGroup)
        .setPosition(c(m.position))
        .setOrientation(c(m.orientation))
        .setSensitivityDb(m.sensitivity)
        .setMaxSplDb(m.maxSpl)
        .setMinSplDb(m.minSpl)
        .setDirectionality(m.directionality)
        .addAllFrequencyResponseHz(m.frequencyResponse.map { it.first })
        .addAllFrequencyResponseDb(m.frequencyResponse.map { it.second })
        .build()
}

/**
 * Over the first [frames] frames, whether every channel is an exact copy of the first: if so, Android gave
 * one microphone several times over, which is said once (the recording itself is kept as it came).
 */
private class ChannelSameness(private val channels: Int, private val frames: Int) {
    private var seen = 0
    private val differs = BooleanArray(channels)
    private val sound = BooleanArray(channels)
    private var done = false

    fun add(buf: ByteArray, n: Int): String? {
        if (done) return null
        val frameBytes = 2 * channels
        var f = 0
        while (f + frameBytes <= n && seen < frames) {
            for (c in 0 until channels) {
                if (buf[f + 2 * c] != 0.toByte() || buf[f + 2 * c + 1] != 0.toByte()) sound[c] = true
                if (c > 0 && (buf[f + 2 * c] != buf[f] || buf[f + 2 * c + 1] != buf[f + 1])) differs[c] = true
            }
            f += frameBytes
            seen++
        }
        if (seen < frames) return null
        done = true
        val silent = (0 until channels).filter { !sound[it] }
        val copies = (1 until channels).filter { !differs[it] && it !in silent }
        val problems = listOfNotNull(
            copies.takeIf { it.isNotEmpty() }?.let { "channels $it were exact copies of channel 0" },
            silent.takeIf { it.isNotEmpty() }?.let { "channels $it were silent" },
        )
        return if (problems.isEmpty()) null else problems.joinToString("; ") + " for the first $frames frames"
    }
}

