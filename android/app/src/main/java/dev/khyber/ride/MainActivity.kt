package dev.khyber.ride

import android.Manifest
import android.content.Intent
import android.content.pm.PackageManager
import android.net.Uri
import android.os.Bundle
import android.os.Environment
import android.os.PowerManager
import android.provider.Settings
import android.util.Log
import androidx.activity.ComponentActivity
import androidx.activity.compose.BackHandler
import androidx.activity.compose.setContent
import androidx.activity.result.contract.ActivityResultContracts
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.FlowRow
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.safeDrawingPadding
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.verticalScroll
import androidx.compose.material3.HorizontalDivider
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
import androidx.compose.ui.text.style.TextOverflow
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
            val v = Units.speed(estimate.speedMps)
            val sigma = Units.speed(estimate.sigmaMps)
            val unit = Units.speedUnit
            // Wraps the average to the next line rather than squeezing it into a column of single letters.
            FlowRow(verticalArrangement = Arrangement.Bottom, horizontalArrangement = Arrangement.spacedBy(16.dp)) {
                Text(
                    when {
                        estimate.stopped -> "0 $unit"
                        // Within its uncertainty of standing still: a number would suggest motion that may not be there.
                        v < sigma -> "< %.0f $unit".format(v + sigma)
                        else -> "%.0f $unit ± %.0f".format(v, sigma)
                    },
                    style = MaterialTheme.typography.displayMedium,
                )
                // The whole ride so far, stops included.
                estimate.averageMps?.let {
                    Text("avg %.0f $unit".format(Units.speed(it)), style = MaterialTheme.typography.headlineMedium)
                }
            }
            // Along the direction of travel: negative is braking.
            Text("%+.2f ${Units.accelUnit}".format(Units.accel(estimate.accelMps2)), style = MaterialTheme.typography.headlineMedium)
            val status by LiveSpeed.status.collectAsStateWithLifecycle()
            status?.let { st ->
                val where = st.station?.let { (if (st.atStation) " · at " else " · near ") + it + st.lines.joinToString("/", " (", ")").takeIf { st.lines.isNotEmpty() }.orEmpty() }.orEmpty()
                Text(st.mode + where + (st.cars?.let { " · $it" } ?: ""), style = MaterialTheme.typography.titleMedium)
            }
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
        var last by remember { mutableStateOf("Nothing marked yet") }
        fun add(kind: ManualEvent.Kind, label: String, text: String = "") {
            last = if (RecorderService.addEvent(kind, text)) "Marked: $label" else "Not recording"
        }
        Column(verticalArrangement = Arrangement.spacedBy(8.dp)) {
            // At the top and one line always, so it can't move the buttons below.
            Text(last, maxLines = 1, overflow = TextOverflow.Ellipsis)
            Text(
                "Tap these as things happen: each marks the moment in the recording, " +
                    "so speeds can be matched to real events afterwards.",
                style = MaterialTheme.typography.bodySmall,
            )
            Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                HapticButton(onClick = { add(ManualEvent.Kind.KIND_BOARDED, "boarded") }, Modifier.weight(1f)) { Text("Boarded") }
                HapticButton(onClick = { add(ManualEvent.Kind.KIND_ALIGHTED, "alighted") }, Modifier.weight(1f)) { Text("Alighted") }
            }
            Text("You got on or off a train.", style = MaterialTheme.typography.bodySmall)
            Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                HapticButton(onClick = { add(ManualEvent.Kind.KIND_TRAIN_STOPPED, "train stopped") }, Modifier.weight(1f)) {
                    Text("Train stopped")
                }
                HapticButton(onClick = { add(ManualEvent.Kind.KIND_TRAIN_STARTED, "train started") }, Modifier.weight(1f)) {
                    Text("Train started")
                }
            }
            Text(
                "The moment the train comes to a stop, and the moment it starts moving again, anywhere, not just at stations.",
                style = MaterialTheme.typography.bodySmall,
            )
            Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                HapticButton(onClick = { add(ManualEvent.Kind.KIND_DOORS_OPEN, "doors opening") }, Modifier.weight(1f)) { Text("Doors opening") }
                HapticButton(onClick = { add(ManualEvent.Kind.KIND_DOORS_CLOSED, "doors closed") }, Modifier.weight(1f)) { Text("Doors closed") }
            }
            Text(
                "Tap as the doors start to open, and once they've finished closing. " +
                    "The train stops a few seconds before the first and leaves a few seconds after the second, " +
                    "so together they bracket each stop.",
                style = MaterialTheme.typography.bodySmall,
            )
            HapticButton(
                onClick = { add(ManualEvent.Kind.KIND_AT_STATION, "at station", note).also { note = "" } },
                Modifier.fillMaxWidth(),
            ) { Text("At station") }
            Text(
                "The train is at a station: type its name below first, so the ride can be matched to the map.",
                style = MaterialTheme.typography.bodySmall,
            )
            OutlinedTextField(note, { note = it }, label = { Text("Station name or note (optional)") }, singleLine = true)
            HapticButton(onClick = { add(ManualEvent.Kind.KIND_NOTE, "note", note).also { note = "" } }) { Text("Add note") }
            Text("Anything else worth knowing later, e.g. \"screen on for beacons\" or \"phone in pocket\".", style = MaterialTheme.typography.bodySmall)
        }
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        Units.load(this)
        Scanning.load(this)
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
                    // The phone's back gesture returns to the main page rather than leaving the app.
                    BackHandler(enabled = page != "main") { page = "main" }
                    when {
                        page == "map" -> LiveMapScreen(onBack = { page = "main" })
                        page == "rides" -> PastRidesScreen(onBack = { page = "main" }, onShowTaps = { id ->
                            // The ride being recorded is still being written: its taps come from the live list.
                            page = if (id == RecorderService.state.value.sessionId) "taps" else "rideTaps:$id"
                        })
                        page == "taps" -> TapsScreen(onBack = { page = "main" })
                        page.startsWith("rideTaps:") -> RideTapsScreen(page.removePrefix("rideTaps:"), onBack = { page = "rides" })
                        else -> Screen(onShowMap = { page = "map" }, onShowRides = { page = "rides" }, onShowTaps = { page = "taps" })
                    }
                }
            }
        }
    }

    @Composable
    private fun Screen(onShowMap: () -> Unit, onShowRides: () -> Unit, onShowTaps: () -> Unit) {
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

        if (!Environment.isExternalStorageManager()) {
            Column(Modifier.padding(24.dp), verticalArrangement = Arrangement.spacedBy(16.dp)) {
                Text("Recordings are kept in shared storage so an uninstall can't delete them; this needs all-files access.")
                HapticButton(onClick = {
                    startActivity(
                        Intent(Settings.ACTION_MANAGE_APP_ALL_FILES_ACCESS_PERMISSION, Uri.parse("package:$packageName")),
                    )
                }) { Text("Grant all-files access") }
            }
            return
        }

        // Two panes that each scroll on their own, so that text changing size in one,
        // like a new problem or another line of speed details, never moves a button in the other under a finger.
        // Clear of the status bar, the navigation bar, and the keyboard.
        Column(Modifier.fillMaxSize().safeDrawingPadding().padding(horizontal = 24.dp)) {
            Column(
                Modifier.weight(1f).verticalScroll(rememberScrollState()).padding(vertical = 24.dp),
                verticalArrangement = Arrangement.spacedBy(16.dp),
            ) {
                // First, so nothing above them can push them down; wrapping, so none is squeezed to nothing.
                FlowRow(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                    HapticButton(onClick = {
                        if (state.recording) RecorderService.stop(this@MainActivity) else RecorderService.start(this@MainActivity)
                    }) { Text(if (state.recording) "Stop" else "Start") }
                    HapticButton(onClick = onShowMap) { Text("Map") }
                    HapticButton(onClick = onShowRides) { Text("Past rides") }
                    HapticButton(onClick = onShowTaps) { Text("Taps") }
                }

                if (!getSystemService(PowerManager::class.java).isIgnoringBatteryOptimizations(packageName)) {
                    Text("Battery optimization can stop recording mid-ride; exempting this app makes that much less likely.")
                    HapticButton(onClick = {
                        startActivity(
                            Intent(Settings.ACTION_REQUEST_IGNORE_BATTERY_OPTIMIZATIONS, Uri.parse("package:$packageName")),
                        )
                    }) { Text("Don't optimize battery for this app") }
                }

                if (checkSelfPermission(Manifest.permission.ACCESS_BACKGROUND_LOCATION) != PackageManager.PERMISSION_GRANTED) {
                    Text("If Android stops and restarts recording, it can only use location with \"Allow all the time\".")
                    HapticButton(onClick = { backgroundLocation.launch(Manifest.permission.ACCESS_BACKGROUND_LOCATION) }) {
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

                Units.Settings()
                Scanning.Settings()

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
                if (state.recording) SpeedDisplay()

                Text(if (auth.configured) oneDriveStatus else "OneDrive: no client ID in this build")
                if (auth.configured) {
                    HapticButton(onClick = {
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
                    HapticButton(onClick = { RecompressWorker.enqueueThenUpload(this@MainActivity) }) { Text("Upload now") }
                }
                Text("App ${BuildConfig.VERSION_NAME}, " + rustCoreStatus, style = MaterialTheme.typography.bodySmall)
            }
            if (state.recording) {
                HorizontalDivider()
                Column(Modifier.weight(2f).verticalScroll(rememberScrollState()).padding(vertical = 16.dp)) { EventButtons() }
            }
        }
    }
}
