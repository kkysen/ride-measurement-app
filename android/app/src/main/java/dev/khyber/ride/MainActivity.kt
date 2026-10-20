package dev.khyber.ride

import android.Manifest
import android.content.Intent
import android.content.pm.PackageManager
import android.net.Uri
import android.os.Bundle
import android.os.Environment
import android.os.PowerManager
import android.util.Log
import android.provider.Settings
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import androidx.activity.result.contract.ActivityResultContracts
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
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
import uniffi.ride_ffi.coreVersion

class MainActivity : ComponentActivity() {
    private val permissions = registerForActivityResult(ActivityResultContracts.RequestMultiplePermissions()) {}

    // Android only grants background location after foreground location, via its own settings screen.
    private val backgroundLocation = registerForActivityResult(ActivityResultContracts.RequestPermission()) {}

    @Composable
    private fun SpeedDisplay() {
        val e by LiveSpeed.estimate.collectAsStateWithLifecycle()
        val error by LiveSpeed.error.collectAsStateWithLifecycle()
        error?.let {
            Text(it, style = MaterialTheme.typography.bodySmall)
            return
        }
        val estimate = e ?: return
        Column {
            val kmh = estimate.speedMps * 3.6
            val sigmaKmh = estimate.sigmaMps * 3.6
            Text(
                when {
                    estimate.stopped -> "0 km/h"
                    // Within its uncertainty of standing still: a number would suggest motion that may not be there.
                    kmh < sigmaKmh -> "under %.0f km/h".format(kmh + sigmaKmh)
                    else -> "%.0f km/h ± %.0f".format(kmh, sigmaKmh)
                },
                style = MaterialTheme.typography.displayMedium,
            )
            // Along the direction of travel: negative is braking.
            Text("%+.2f m/s²".format(estimate.accelMps2), style = MaterialTheme.typography.headlineMedium)
            val sensors = estimate.sensors.sortedBy { it.ordinal }.joinToString { it.name.lowercase().replace('_', ' ') }
            Text(if (estimate.stopped) "stopped · $sensors" else sensors)
            val gnss by LiveSpeed.gnss.collectAsStateWithLifecycle()
            gnss?.let { g ->
                Text(
                    "raw GNSS: ${g.epochsSolved} of ${g.epochs} epochs solved, ${g.ephemerides} orbits " +
                        "(${g.subframesOk} navigation subframes, ${g.subframesBad} failed parity)",
                    style = MaterialTheme.typography.bodySmall,
                )
            }
            Text(estimate.sinceGnssS?.let { if (it < 3) "GNSS: live" else "no GNSS for %.0f s".format(it) } ?: "no GNSS fix yet")
            if (estimate.rejected.isNotEmpty()) {
                Text("rejected as outliers: " + estimate.rejected.joinToString { "${it.sensor.name.lowercase().replace('_', ' ')} ${it.count}" })
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
            Text(
                "Tap these as things happen: each marks the moment in the recording, " +
                    "so speeds can be matched to real events afterwards.",
                style = MaterialTheme.typography.bodySmall,
            )
            Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                Button(onClick = { add(ManualEvent.Kind.KIND_BOARDED, "boarded") }, Modifier.weight(1f)) { Text("Boarded") }
                Button(onClick = { add(ManualEvent.Kind.KIND_ALIGHTED, "alighted") }, Modifier.weight(1f)) { Text("Alighted") }
            }
            Text("You got on or off a train.", style = MaterialTheme.typography.bodySmall)
            Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                Button(onClick = { add(ManualEvent.Kind.KIND_DOORS_OPEN, "doors open") }, Modifier.weight(1f)) { Text("Doors open") }
                Button(onClick = { add(ManualEvent.Kind.KIND_DOORS_CLOSED, "doors closed") }, Modifier.weight(1f)) { Text("Doors closed") }
            }
            Text(
                "The train's doors opened or closed: marks exactly when it stopped and is about to leave.",
                style = MaterialTheme.typography.bodySmall,
            )
            Button(
                onClick = { add(ManualEvent.Kind.KIND_AT_STATION, "at station", note).also { note = "" } },
                Modifier.fillMaxWidth(),
            ) { Text("At station") }
            Text(
                "The train is at a station: type its name below first, so the ride can be matched to the map.",
                style = MaterialTheme.typography.bodySmall,
            )
            OutlinedTextField(note, { note = it }, label = { Text("Station name or note (optional)") })
            Button(onClick = { add(ManualEvent.Kind.KIND_NOTE, "note", note).also { note = "" } }) { Text("Add note") }
            Text("Anything else worth knowing later, e.g. \"screen on for beacons\" or \"phone in pocket\".", style = MaterialTheme.typography.bodySmall)
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
                Manifest.permission.BLUETOOTH_SCAN,
                Manifest.permission.RECORD_AUDIO,
            ),
        )
        setContent {
            MaterialTheme {
                Surface(Modifier.fillMaxSize()) {
                    var page by remember { mutableStateOf("main") }
                    when (page) {
                        "map" -> LiveMapScreen(onBack = { page = "main" })
                        "rides" -> PastRidesScreen(onBack = { page = "main" })
                        else -> Screen(onShowMap = { page = "map" }, onShowRides = { page = "rides" })
                    }
                }
            }
        }
    }

    @Composable
    private fun Screen(onShowMap: () -> Unit, onShowRides: () -> Unit) {
        // Recording doesn't use the Rust core, so if it fails to load, show why and keep going.
        val rustCoreStatus = remember {
            try {
                "Rust core ${coreVersion()}"
            } catch (e: Throwable) {
                Log.e("RideLogger", "Rust core failed to load", e)
                "Rust core failed to load (recording still works):\n${Log.getStackTraceString(e)}"
            }
        }
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
            state.error?.let { Text(it, style = MaterialTheme.typography.bodySmall) }
            // Parts that failed while everything else kept recording (see `Health`).
            val problems by Health.problems.collectAsStateWithLifecycle()
            if (problems.isNotEmpty()) {
                Text(
                    "Not recording fully:\n" + problems.entries.joinToString("\n") { (source, p) ->
                        "$source: ${p.message}" + if (p.count > 1) " (${p.count} times)" else ""
                    },
                    style = MaterialTheme.typography.bodySmall,
                    color = MaterialTheme.colorScheme.error,
                )
            }
            Button(onClick = {
                if (state.recording) RecorderService.stop(this@MainActivity) else RecorderService.start(this@MainActivity)
            }) { Text(if (state.recording) "Stop" else "Start") }
            Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                Button(onClick = onShowMap) { Text("Map") }
                Button(onClick = onShowRides) { Text("Past rides") }
            }

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
            Text("App ${BuildConfig.VERSION_NAME}, " + rustCoreStatus, style = MaterialTheme.typography.bodySmall)
        }
    }
}
