package dev.khyber.ride

import android.Manifest
import android.app.Notification
import android.app.NotificationChannel
import android.app.NotificationManager
import android.app.Service
import android.content.Context
import android.content.Intent
import android.content.pm.PackageManager
import android.content.pm.ServiceInfo
import android.hardware.Sensor
import android.hardware.SensorManager
import android.location.LocationManager
import android.os.Build
import android.os.Environment
import android.os.IBinder
import android.os.PowerManager
import android.os.SystemClock
import dev.khyber.ride.proto.Device
import dev.khyber.ride.proto.ManualEvent
import dev.khyber.ride.proto.SessionInfo
import java.io.File
import java.text.SimpleDateFormat
import java.util.Date
import java.util.Locale
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow

/** Foreground service that keeps recording with the screen off. */
class RecorderService : Service() {
    private var wakeLock: PowerManager.WakeLock? = null
    private var recorders: List<SensorStreamRecorder> = emptyList()
    private var recordStreams: List<RecordStreamRecorder> = emptyList()
    private var gnssRecorder: GnssRecorder? = null
    private var radioRecorder: RadioRecorder? = null
    private var audioRecorder: AudioStreamRecorder? = null

    override fun onBind(intent: Intent?): IBinder? = null

    override fun onStartCommand(intent: Intent?, flags: Int, startId: Int): Int {
        if (recorders.isNotEmpty()) return START_STICKY
        val prefs = getSharedPreferences(PREFS, MODE_PRIVATE)
        // A null intent means Android killed the service and restarted it (`START_STICKY`).
        val restarted = intent == null
        if (restarted && !prefs.getBoolean(WANTED, false)) {
            stopSelf()
            return START_NOT_STICKY
        }
        prefs.edit().putBoolean(WANTED, true).commit()
        // The killed session's files can't be safely appended to, so a restart starts a new session that links back.
        val continues = if (restarted) prefs.getString(LAST_SESSION, null).orEmpty() else ""
        // Started from the background (a restart) without "Allow all the time", Android doesn't allow a location service,
        // so fall back to recording everything that doesn't need location.
        val locationAllowed = !restarted ||
            checkSelfPermission(Manifest.permission.ACCESS_BACKGROUND_LOCATION) == PackageManager.PERMISSION_GRANTED
        // The microphone also can't be started from the background, so a restarted session records no audio.
        val audioAllowed = !restarted && AudioStreamRecorder.enabled(this) &&
            checkSelfPermission(Manifest.permission.RECORD_AUDIO) == PackageManager.PERMISSION_GRANTED
        var serviceType = if (locationAllowed) {
            ServiceInfo.FOREGROUND_SERVICE_TYPE_LOCATION
        } else {
            ServiceInfo.FOREGROUND_SERVICE_TYPE_SPECIAL_USE
        }
        if (audioAllowed) serviceType = serviceType or ServiceInfo.FOREGROUND_SERVICE_TYPE_MICROPHONE
        startForeground(NOTIFICATION_ID, notification(locationAllowed), serviceType)
        wakeLock = getSystemService(PowerManager::class.java)
            .newWakeLock(PowerManager.PARTIAL_WAKE_LOCK, "ride:recording")
            .apply { acquire() }

        val startUnixMs = System.currentTimeMillis()
        val startElapsedNs = SystemClock.elapsedRealtimeNanos()
        val base = SimpleDateFormat("yyyyMMdd-HHmmss", Locale.US).format(Date(startUnixMs))
        val id = generateSequence(1) { it + 1 }.map { if (it == 1) base else "$base-$it" }
            .first { !File(sessionsDir(), it).exists() }
        val dir = File(sessionsDir(), id).apply { mkdirs() }
        prefs.edit().putString(LAST_SESSION, id).commit()

        val sensorManager = getSystemService(SensorManager::class.java)
        // Uncalibrated variants carry the raw values plus the bias estimate, so calibrated ones are derivable and not stored;
        // calibrated is the fallback only where uncalibrated doesn't exist.
        val sensorSpecs = listOf(
            SensorSpec("accel_uncal", Sensor.TYPE_ACCELEROMETER_UNCALIBRATED, fallback = SensorSpec("accel", Sensor.TYPE_ACCELEROMETER)),
            SensorSpec("gyro_uncal", Sensor.TYPE_GYROSCOPE_UNCALIBRATED, fallback = SensorSpec("gyro", Sensor.TYPE_GYROSCOPE)),
            SensorSpec("mag_uncal", Sensor.TYPE_MAGNETIC_FIELD_UNCALIBRATED, fallback = SensorSpec("mag", Sensor.TYPE_MAGNETIC_FIELD)),
            SensorSpec("pressure", Sensor.TYPE_PRESSURE),
            SensorSpec("light", Sensor.TYPE_LIGHT),
            SensorSpec("proximity", Sensor.TYPE_PROXIMITY),
        ).mapNotNull { spec ->
            sequenceOf(spec, spec.fallback).filterNotNull()
                .firstNotNullOfOrNull { s -> sensorManager.getDefaultSensor(s.type)?.let { s to it } }
        }
        val highRate = sensorSpecs.count { (_, sensor) -> sensor.reportingMode == Sensor.REPORTING_MODE_CONTINUOUS }
        recorders = sensorSpecs.map { (spec, sensor) ->
            SensorStreamRecorder(sensorManager, sensor, spec.name, dir, sharingFifo = highRate)
        }
        val events = RecordStreamRecorder("events", dir)
        val radio = RecordStreamRecorder("radio", dir)
        recordStreams = listOf(events, radio)
        if (locationAllowed) {
            val gnss = RecordStreamRecorder("gnss", dir)
            val locations = RecordStreamRecorder("location", dir)
            recordStreams = recordStreams + listOf(gnss, locations)
            gnssRecorder = GnssRecorder(getSystemService(LocationManager::class.java), gnss, locations)
        }
        val gnssRef = gnssRecorder
        radioRecorder = RadioRecorder(this, radio) { gnssRef?.lastGpsFixElapsedNs ?: 0L }
        eventsStream = events
        if (audioAllowed) audioRecorder = AudioStreamRecorder(this, dir)
        SessionWriter.writeSessionInfo(
            dir,
            SessionInfo.newBuilder()
                .setFormatVersion(FORMAT_VERSION)
                .setSessionId(id)
                .setStartUnixMs(startUnixMs)
                .setStartElapsedRealtimeNs(startElapsedNs)
                .setContinuesSessionId(continues)
                .setDevice(
                    Device.newBuilder()
                        .setManufacturer(Build.MANUFACTURER)
                        .setModel(Build.MODEL)
                        .setSdkInt(Build.VERSION.SDK_INT)
                        .setBuildFingerprint(Build.FINGERPRINT),
                )
                .addAllStreams(recorders.map { it.streamInfo })
                .addAllStreams(recordStreams.map { it.streamInfo })
                .addAllStreams(listOfNotNull(audioRecorder?.streamInfo))
                .build(),
        )
        LiveTrack.reset()
        LiveSpeed.start()
        recorders.forEach { it.start() }
        try {
            gnssRecorder?.start()
        } catch (e: SecurityException) {
            android.util.Log.w("RecorderService", "no location permission; not recording GNSS", e)
        }
        radioRecorder?.start()
        audioRecorder?.start()
        if (restarted) {
            events.add {
                manualEvent = ManualEvent.newBuilder()
                    .setKind(ManualEvent.Kind.KIND_NOTE)
                    .setText("restarted after Android stopped recording; location ${if (locationAllowed) "on" else "off"}")
                    .build()
            }
        }
        _state.value = State(recording = true, sessionId = id)
        return START_STICKY
    }

    override fun onDestroy() {
        eventsStream = null
        gnssRecorder?.stop()
        gnssRecorder = null
        radioRecorder?.stop()
        radioRecorder = null
        audioRecorder?.stop()
        audioRecorder = null
        LiveSpeed.stop()
        recorders.forEach { it.beginStop() }
        recorders.forEach { it.stop() }
        recorders = emptyList()
        recordStreams.forEach { it.stop() }
        recordStreams = emptyList()
        wakeLock?.release()
        _state.value = State(recording = false, sessionId = null)
        RecompressWorker.enqueueThenUpload(this)
        super.onDestroy()
    }

    private fun notification(locationAllowed: Boolean): Notification {
        val manager = getSystemService(NotificationManager::class.java)
        manager.createNotificationChannel(
            NotificationChannel(CHANNEL_ID, "Recording", NotificationManager.IMPORTANCE_LOW),
        )
        return Notification.Builder(this, CHANNEL_ID)
            .setContentTitle(if (locationAllowed) "Recording ride" else "Recording ride without location")
            .setSmallIcon(android.R.drawable.presence_video_online)
            .setOngoing(true)
            .build()
    }

    data class State(val recording: Boolean, val sessionId: String?)

    private data class SensorSpec(val name: String, val type: Int, val fallback: SensorSpec? = null)

    companion object {
        private const val CHANNEL_ID = "recording"
        private const val NOTIFICATION_ID = 1
        private const val PREFS = "recorder"

        /** Whether the user wants to be recording; only Stop clears it, so a kill by Android restarts recording. */
        private const val WANTED = "wanted"
        private const val LAST_SESSION = "last_session"

        fun start(context: Context) {
            context.startForegroundService(Intent(context, RecorderService::class.java))
        }

        fun stop(context: Context) {
            context.getSharedPreferences(PREFS, MODE_PRIVATE).edit().putBoolean(WANTED, false).commit()
            context.stopService(Intent(context, RecorderService::class.java))
        }

        @Volatile
        private var eventsStream: RecordStreamRecorder? = null

        /** Records a manual event (boarded, at station, ...) into the current session; false if not recording. */
        fun addEvent(kind: ManualEvent.Kind, text: String = ""): Boolean {
            val events = eventsStream ?: return false
            events.add { manualEvent = ManualEvent.newBuilder().setKind(kind).setText(text).build() }
            return true
        }

        private val _state = MutableStateFlow(State(recording = false, sessionId = null))
        val state: StateFlow<State> = _state

        /**
         * Shared storage, not app-private storage, so recordings survive an uninstall or a reinstall;
         * needs all-files access (`MANAGE_EXTERNAL_STORAGE`).
         */
        fun sessionsDir() = File(Environment.getExternalStorageDirectory(), "RideLogger/sessions")
    }
}
