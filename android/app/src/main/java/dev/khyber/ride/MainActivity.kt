package dev.khyber.ride

import android.Manifest
import android.content.Intent
import android.content.pm.PackageManager
import android.net.Uri
import android.os.Bundle
import android.os.Environment
import android.os.PowerManager
import android.provider.Settings
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import androidx.activity.result.contract.ActivityResultContracts
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.verticalScroll
import androidx.compose.material3.Button
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.OutlinedTextField
import androidx.compose.material3.Surface
import androidx.compose.material3.Switch
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.rememberCoroutineScope
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.unit.dp
import androidx.lifecycle.compose.collectAsStateWithLifecycle
import dev.khyber.ride.onedrive.OneDriveAuth
import dev.khyber.ride.proto.ManualEvent
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.launch
import kotlinx.coroutines.withContext

class MainActivity : ComponentActivity() {
    private val permissions = registerForActivityResult(ActivityResultContracts.RequestMultiplePermissions()) {}

    // Android only grants background location after foreground location, via its own settings screen.
    private val backgroundLocation = registerForActivityResult(ActivityResultContracts.RequestPermission()) {}

    @Composable
    private fun SpeedDisplay() {
        val e by LiveSpeed.estimate.collectAsStateWithLifecycle()
        val estimate = e ?: return
        Column {
            Text(
                "%.0f km/h ± %.0f".format(estimate.speedMps * 3.6, estimate.sigmaMps * 3.6),
                style = MaterialTheme.typography.displayMedium,
            )
            val sensors = estimate.sensors.sortedBy { it.ordinal }.joinToString { it.name.lowercase().replace('_', ' ') }
            Text(if (estimate.stopped) "stopped · $sensors" else sensors)
            Text(estimate.sinceGnssS?.let { if (it < 3) "GNSS: live" else "no GNSS for %.0f s".format(it) } ?: "no GNSS fix yet")
            if (estimate.rejected.isNotEmpty()) {
                Text("rejected as outliers: " + estimate.rejected.entries.joinToString { "${it.key.name.lowercase()} ${it.value}" })
            }
        }
    }

    @Composable
    private fun EventButtons() {
        var note by remember { mutableStateOf("") }
        var last by remember { mutableStateOf("") }
        fun add(kind: ManualEvent.Kind, label: String, text: String = "") {
            last = if (RecorderService.addEvent(kind, text)) "Marked: $label" else "Not recording"
        }
        Column(verticalArrangement = Arrangement.spacedBy(8.dp)) {
            Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                Button(onClick = { add(ManualEvent.Kind.KIND_BOARDED, "boarded") }) { Text("Boarded") }
                Button(onClick = { add(ManualEvent.Kind.KIND_ALIGHTED, "alighted") }) { Text("Alighted") }
            }
            Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                Button(onClick = { add(ManualEvent.Kind.KIND_AT_STATION, "at station", note).also { note = "" } }) {
                    Text("At station")
                }
                Button(onClick = { add(ManualEvent.Kind.KIND_DOORS_OPEN, "doors open") }) { Text("Doors open") }
                Button(onClick = { add(ManualEvent.Kind.KIND_DOORS_CLOSED, "doors closed") }) { Text("Doors closed") }
            }
            OutlinedTextField(note, { note = it }, label = { Text("Station name or note (optional)") })
            Button(onClick = { add(ManualEvent.Kind.KIND_NOTE, "note", note).also { note = "" } }) { Text("Add note") }
            if (last.isNotEmpty()) Text(last)
        }
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        permissions.launch(
            arrayOf(
                Manifest.permission.ACCESS_FINE_LOCATION,
                Manifest.permission.ACCESS_COARSE_LOCATION,
                Manifest.permission.POST_NOTIFICATIONS,
                Manifest.permission.READ_PHONE_STATE,
                Manifest.permission.NEARBY_WIFI_DEVICES,
                Manifest.permission.RECORD_AUDIO,
            ),
        )
        setContent {
            MaterialTheme {
                Surface(Modifier.fillMaxSize()) { Screen() }
            }
        }
    }

    @Composable
    private fun Screen() {
        val state by RecorderService.state.collectAsStateWithLifecycle()
        val auth = remember { OneDriveAuth(this) }
        var oneDriveStatus by remember { mutableStateOf(if (auth.signedIn) "OneDrive: signed in" else "OneDrive: not signed in") }
        val scope = rememberCoroutineScope()

        Column(
            Modifier.padding(24.dp).verticalScroll(rememberScrollState()),
            verticalArrangement = Arrangement.spacedBy(16.dp),
        ) {
            if (!Environment.isExternalStorageManager()) {
                Text("Recordings are kept in shared storage so an uninstall can't delete them; this needs all-files access.")
                Button(onClick = {
                    startActivity(
                        Intent(Settings.ACTION_MANAGE_APP_ALL_FILES_ACCESS_PERMISSION, Uri.parse("package:$packageName")),
                    )
                }) { Text("Grant all-files access") }
                return@Column
            }

            if (!getSystemService(PowerManager::class.java).isIgnoringBatteryOptimizations(packageName)) {
                Text("Battery optimization can stop recording mid-ride; exempting this app makes that much less likely.")
                Button(onClick = {
                    startActivity(
                        Intent(Settings.ACTION_REQUEST_IGNORE_BATTERY_OPTIMIZATIONS, Uri.parse("package:$packageName")),
                    )
                }) { Text("Don't optimize battery for this app") }
            }

            if (checkSelfPermission(Manifest.permission.ACCESS_BACKGROUND_LOCATION) != PackageManager.PERMISSION_GRANTED) {
                Text("If Android stops and restarts recording, it can only use location with \"Allow all the time\".")
                Button(onClick = { backgroundLocation.launch(Manifest.permission.ACCESS_BACKGROUND_LOCATION) }) {
                    Text("Allow location all the time")
                }
            }

            var audio by remember { mutableStateOf(AudioStreamRecorder.enabled(this@MainActivity)) }
            Row(verticalAlignment = Alignment.CenterVertically, horizontalArrangement = Arrangement.spacedBy(12.dp)) {
                Switch(
                    checked = audio,
                    enabled = !state.recording,
                    onCheckedChange = {
                        audio = it
                        AudioStreamRecorder.setEnabled(this@MainActivity, it)
                    },
                )
                Text("Record audio (lossless, ~100 MB/h; also records conversations nearby)")
            }

            Text(if (state.recording) "Recording ${state.sessionId}" else "Not recording")
            Button(onClick = {
                if (state.recording) RecorderService.stop(this@MainActivity) else RecorderService.start(this@MainActivity)
            }) { Text(if (state.recording) "Stop" else "Start") }

            if (state.recording) SpeedDisplay()
            if (state.recording) EventButtons()

            Text(if (auth.configured) oneDriveStatus else "OneDrive: no client ID in this build")
            if (auth.configured) {
                Button(onClick = {
                    scope.launch {
                        try {
                            val code = withContext(Dispatchers.IO) { auth.startDeviceCode() }
                            oneDriveStatus = "Go to ${code.verificationUri} and enter ${code.userCode}"
                            withContext(Dispatchers.IO) { auth.awaitDeviceCode(code) }
                            oneDriveStatus = "OneDrive: signed in"
                            RecompressWorker.enqueueThenUpload(this@MainActivity)
                        } catch (e: Exception) {
                            oneDriveStatus = "OneDrive sign-in failed: ${e.message}"
                        }
                    }
                }) { Text(if (auth.signedIn) "Sign in again" else "Sign in to OneDrive") }
                Button(onClick = { RecompressWorker.enqueueThenUpload(this@MainActivity) }) { Text("Upload now") }
            }
        }
    }
}
