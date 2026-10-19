package dev.kkysen.ride

import android.app.Notification
import android.app.NotificationChannel
import android.app.NotificationManager
import android.app.Service
import android.content.Intent
import android.content.pm.ServiceInfo
import android.hardware.Sensor
import android.hardware.SensorManager
import android.os.Build
import android.os.Environment
import android.os.IBinder
import android.os.PowerManager
import android.os.SystemClock
import dev.kkysen.ride.proto.Device
import dev.kkysen.ride.proto.SessionInfo
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

    override fun onBind(intent: Intent?): IBinder? = null

    override fun onStartCommand(intent: Intent?, flags: Int, startId: Int): Int {
        if (recorders.isNotEmpty()) return START_NOT_STICKY
        startForeground(NOTIFICATION_ID, notification(), ServiceInfo.FOREGROUND_SERVICE_TYPE_LOCATION)
        wakeLock = getSystemService(PowerManager::class.java)
            .newWakeLock(PowerManager.PARTIAL_WAKE_LOCK, "ride:recording")
            .apply { acquire() }

        val startUnixMs = System.currentTimeMillis()
        val startElapsedNs = SystemClock.elapsedRealtimeNanos()
        val id = SimpleDateFormat("yyyyMMdd-HHmmss", Locale.US).format(Date(startUnixMs))
        val dir = File(sessionsDir(), id).apply { mkdirs() }

        val sensorManager = getSystemService(SensorManager::class.java)
        recorders = listOfNotNull(
            sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER)?.let {
                SensorStreamRecorder(sensorManager, it, "accel", 3, dir)
            },
        )
        SessionWriter.writeSessionInfo(
            dir,
            SessionInfo.newBuilder()
                .setFormatVersion(FORMAT_VERSION)
                .setSessionId(id)
                .setStartUnixMs(startUnixMs)
                .setStartElapsedRealtimeNs(startElapsedNs)
                .setDevice(
                    Device.newBuilder()
                        .setManufacturer(Build.MANUFACTURER)
                        .setModel(Build.MODEL)
                        .setSdkInt(Build.VERSION.SDK_INT)
                        .setBuildFingerprint(Build.FINGERPRINT),
                )
                .addAllStreams(recorders.map { it.streamInfo })
                .build(),
        )
        recorders.forEach { it.start() }
        _state.value = State(recording = true, sessionId = id)
        return START_NOT_STICKY
    }

    override fun onDestroy() {
        recorders.forEach { it.stop() }
        recorders = emptyList()
        wakeLock?.release()
        _state.value = State(recording = false, sessionId = null)
        UploadWorker.enqueue(this)
        super.onDestroy()
    }

    private fun notification(): Notification {
        val manager = getSystemService(NotificationManager::class.java)
        manager.createNotificationChannel(
            NotificationChannel(CHANNEL_ID, "Recording", NotificationManager.IMPORTANCE_LOW),
        )
        return Notification.Builder(this, CHANNEL_ID)
            .setContentTitle("Recording ride")
            .setSmallIcon(android.R.drawable.presence_video_online)
            .setOngoing(true)
            .build()
    }

    data class State(val recording: Boolean, val sessionId: String?)

    companion object {
        private const val CHANNEL_ID = "recording"
        private const val NOTIFICATION_ID = 1
        private val _state = MutableStateFlow(State(recording = false, sessionId = null))
        val state: StateFlow<State> = _state

        /**
         * Shared storage, not app-private storage, so recordings survive an uninstall or a reinstall;
         * needs all-files access (`MANAGE_EXTERNAL_STORAGE`).
         */
        fun sessionsDir() = File(Environment.getExternalStorageDirectory(), "RideLogger/sessions")
    }
}
