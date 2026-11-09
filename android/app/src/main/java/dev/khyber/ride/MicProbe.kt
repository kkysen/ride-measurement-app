package dev.khyber.ride

import android.annotation.SuppressLint
import android.app.Notification
import android.app.NotificationChannel
import android.app.NotificationManager
import android.app.Service
import android.content.Context
import android.content.Intent
import android.content.pm.ServiceInfo
import android.media.AudioDeviceInfo
import android.media.AudioFormat
import android.media.AudioManager
import android.media.AudioRecord
import android.media.MediaRecorder
import android.os.IBinder
import android.util.Log
import java.io.File
import kotlin.math.log10
import kotlin.math.sqrt

/**
 * Tries every audio source, input device and channel layout for a moment each, and writes what each really
 * gave into `mic-probe.tsv` in the app's files directory: the channels Android opened, which microphones it
 * says were active, each channel's level, and which channels were copies of another or silent.
 * Only run on request (`adb shell am start -n dev.khyber.ride/.MainActivity --ez probe_mics true`),
 * never while recording a ride, to choose what [AudioStreamRecorder] opens.
 * It runs in [MicProbeService], a foreground service, since Android silences the microphone of an app
 * that's left the screen: the phone can be locked or used for something else meanwhile.
 */
@SuppressLint("MissingPermission") // Only run after `RECORD_AUDIO` is granted.
object MicProbe {
    private const val TAG = "MicProbe"
    private const val RATE = 48_000

    private val sources = listOf(
        "DEFAULT" to MediaRecorder.AudioSource.DEFAULT,
        "MIC" to MediaRecorder.AudioSource.MIC,
        "CAMCORDER" to MediaRecorder.AudioSource.CAMCORDER,
        "VOICE_RECOGNITION" to MediaRecorder.AudioSource.VOICE_RECOGNITION,
        "VOICE_COMMUNICATION" to MediaRecorder.AudioSource.VOICE_COMMUNICATION,
        "UNPROCESSED" to MediaRecorder.AudioSource.UNPROCESSED,
        "VOICE_PERFORMANCE" to MediaRecorder.AudioSource.VOICE_PERFORMANCE,
    )

    private fun layouts(): List<Pair<String, AudioFormat.Builder.() -> AudioFormat.Builder>> =
        (1..4).map { n -> "index$n" to { b: AudioFormat.Builder -> b.setChannelIndexMask((1 shl n) - 1) } } + listOf(
            "mono" to { setChannelMask(AudioFormat.CHANNEL_IN_MONO) },
            "stereo" to { setChannelMask(AudioFormat.CHANNEL_IN_STEREO) },
            "front_back" to { setChannelMask(AudioFormat.CHANNEL_IN_FRONT or AudioFormat.CHANNEL_IN_BACK) },
        )

    fun run(context: Context) {
        context.startForegroundService(Intent(context, MicProbeService::class.java))
    }

    /** Runs the probe, then stops [service]. */
    fun start(service: Service) {
        Thread({
            Health.guard("mic probe") { probe(service) }
            service.stopSelf()
        }, "mic probe").start()
    }

    private fun probe(context: Context) {
        val am = context.getSystemService(AudioManager::class.java)
        val out = File(context.getExternalFilesDir(null), "mic-probe.tsv")
        val inputs = am.getDevices(AudioManager.GET_DEVICES_INPUTS)
            .filter { it.type == AudioDeviceInfo.TYPE_BUILTIN_MIC || it.type == AudioDeviceInfo.TYPE_TELEPHONY }
        val devices = listOf<AudioDeviceInfo?>(null) + inputs.filter { it.type == AudioDeviceInfo.TYPE_BUILTIN_MIC }
        out.printWriter().use { w ->
            w.println("# unprocessed supported: ${am.getProperty(AudioManager.PROPERTY_SUPPORT_AUDIO_SOURCE_UNPROCESSED)}")
            for (d in inputs) {
                w.println("# device ${d.id} type ${d.type} '${d.productName}' address '${d.address}' channels ${d.channelCounts.toList()} index masks ${d.channelIndexMasks.toList()} masks ${d.channelMasks.toList()} encodings ${d.encodings.toList()}")
            }
            for (m in am.microphones) w.println("# microphone ${m.id} '${m.description}' type ${m.type} address '${m.address}' position ${m.position.x},${m.position.y},${m.position.z} orientation ${m.orientation.x},${m.orientation.y},${m.orientation.z}")
            w.println("source\tdevice\tlayout\tencoding\tresult\tchannels\trouted\tactive_mics\tdbfs\tcopies\tsilent")
            for ((sourceName, source) in sources) for (device in devices) for ((layoutName, layout) in layouts()) {
                for ((encName, enc) in listOf("s16" to AudioFormat.ENCODING_PCM_16BIT, "float" to AudioFormat.ENCODING_PCM_FLOAT)) {
                    val deviceName = device?.address?.ifEmpty { device.productName.toString() } ?: "default"
                    val row = Health.guard("mic probe row") { one(source, device, layout, enc) } ?: "error"
                    w.println("$sourceName\t$deviceName\t$layoutName\t$encName\t$row")
                    w.flush()
                }
            }
        }
        Log.i(TAG, "done: $out")
    }

    private fun one(source: Int, device: AudioDeviceInfo?, layout: AudioFormat.Builder.() -> AudioFormat.Builder, enc: Int): String {
        val format = try {
            AudioFormat.Builder().setEncoding(enc).setSampleRate(RATE).layout().build()
        } catch (e: Exception) {
            return "bad format: $e"
        }
        val record = try {
            AudioRecord.Builder().setAudioSource(source).setAudioFormat(format).setBufferSizeInBytes(RATE * 4 * 4).build()
        } catch (e: Exception) {
            return "refused: $e"
        }
        try {
            if (record.state != AudioRecord.STATE_INITIALIZED) return "not initialized"
            device?.let { record.setPreferredDevice(it) }
            val ch = record.channelCount
            record.startRecording()
            // The first moment is often silent or ramping while the input starts, so it's read and dropped.
            val frames = RATE / 2
            val data = Array(ch) { FloatArray(frames) }
            var routed = ""
            var active = ""
            for (pass in 0..1) {
                var got = 0
                while (got < frames) {
                    val n = if (enc == AudioFormat.ENCODING_PCM_FLOAT) {
                        val b = FloatArray((frames - got) * ch)
                        val r = record.read(b, 0, b.size, AudioRecord.READ_BLOCKING)
                        if (r <= 0) return "read $r"
                        for (i in 0 until r / ch) for (c in 0 until ch) data[c][got + i] = b[i * ch + c]
                        r / ch
                    } else {
                        val b = ShortArray((frames - got) * ch)
                        val r = record.read(b, 0, b.size)
                        if (r <= 0) return "read $r"
                        for (i in 0 until r / ch) for (c in 0 until ch) data[c][got + i] = b[i * ch + c] / 32768f
                        r / ch
                    }
                    got += n
                }
                if (pass == 0) {
                    routed = record.routedDevice?.let { "${it.type}:${it.address}" } ?: ""
                    active = Health.guard("active mics") { record.activeMicrophones.joinToString(",") { "${it.id}:${it.address}" } } ?: "?"
                }
            }
            record.stop()
            val dbfs = data.map { x -> 20 * log10(sqrt(x.sumOf { (it * it).toDouble() } / x.size) + 1e-12) }
            val copies = (1 until ch).mapNotNull { c -> (0 until c).firstOrNull { data[it].contentEquals(data[c]) }?.let { "$c=$it" } }
            val silent = (0 until ch).filter { c -> data[c].all { it == 0f } }
            return "ok\t$ch\t$routed\t$active\t${dbfs.joinToString(",") { "%.1f".format(it) }}\t${copies.joinToString(",")}\t${silent.joinToString(",")}"
        } finally {
            record.release()
        }
    }
}

/** Keeps the microphone usable for [MicProbe] while the app isn't on screen. */
class MicProbeService : Service() {
    private var started = false

    override fun onBind(intent: Intent?): IBinder? = null

    override fun onStartCommand(intent: Intent?, flags: Int, startId: Int): Int {
        val manager = getSystemService(NotificationManager::class.java)
        manager.createNotificationChannel(NotificationChannel(CHANNEL_ID, "Microphone check", NotificationManager.IMPORTANCE_LOW))
        val notification = Notification.Builder(this, CHANNEL_ID)
            .setContentTitle("Checking the microphones (about 7 min)")
            .setSmallIcon(android.R.drawable.ic_btn_speak_now)
            .setOngoing(true)
            .build()
        startForeground(NOTIFICATION_ID, notification, ServiceInfo.FOREGROUND_SERVICE_TYPE_MICROPHONE)
        if (!started) {
            started = true
            MicProbe.start(this)
        }
        return START_NOT_STICKY
    }

    private companion object {
        const val CHANNEL_ID = "mic probe"
        const val NOTIFICATION_ID = 3
    }
}
