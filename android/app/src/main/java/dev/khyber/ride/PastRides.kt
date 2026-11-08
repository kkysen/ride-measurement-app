package dev.khyber.ride

import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.safeDrawingPadding
import androidx.compose.foundation.layout.width
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.verticalScroll
import androidx.compose.material3.LinearProgressIndicator
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateMapOf
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.unit.dp
import androidx.lifecycle.compose.collectAsStateWithLifecycle
import android.os.SystemClock
import android.util.Log
import java.io.File
import java.time.Instant
import java.time.ZoneId
import java.time.format.DateTimeFormatter
import java.util.Locale
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.withContext
import uniffi.ride_ffi.ReplayProgress
import uniffi.ride_ffi.RideReplay
import uniffi.ride_ffi.SessionSpan
import uniffi.ride_ffi.cachedReplay
import uniffi.ride_ffi.replaySession
import uniffi.ride_ffi.sessionMarks
import uniffi.ride_ffi.sessionSpan
import dev.khyber.ride.proto.ManualEvent

/** One recording on the phone, as far as its files tell without decoding them. */
private data class PastRide(val id: String, val bytes: Long, val recompressed: Boolean)

private val NEW_YORK = ZoneId.of("America/New_York")
private val DATE = DateTimeFormatter.ofPattern("EEE MMM d, yyyy", Locale.US)
private val TIME = DateTimeFormatter.ofPattern("h:mm:ss a", Locale.US)

/** A ride's start and end in New York time, the date once if both are on the same day, and how long: h:mm:ss. */
private fun describeSpan(span: SessionSpan): String {
    val start = Instant.ofEpochMilli(span.startUnixMs).atZone(NEW_YORK)
    val time = { t: java.time.ZonedDateTime -> TIME.format(t).lowercase() }
    val endMs = span.endUnixMs ?: return "${DATE.format(start)}, ${time(start)}"
    val end = Instant.ofEpochMilli(endMs).atZone(NEW_YORK)
    val until = if (end.toLocalDate() == start.toLocalDate()) time(end) else "${DATE.format(end)}, ${time(end)}"
    val total = ((endMs - span.startUnixMs) / 1000).coerceAtLeast(0)
    return "${DATE.format(start)}, ${time(start)} to $until · " +
        "%d:%02d:%02d".format(total / 3600, total / 60 % 60, total % 60)
}

private fun pastRides(): List<PastRide> =
    RecorderService.sessionsDir().listFiles().orEmpty().filter { it.isDirectory }.map { dir ->
        val files = dir.walkTopDown().filter { it.isFile }.toList()
        PastRide(
            id = dir.name,
            bytes = files.sumOf { it.length() },
            recompressed = File(dir, ".recompressed").exists(),
        )
    }.sortedByDescending { it.id }

/** The recordings on the phone, newest first: when they started and ended, how long, and how big. */
@Composable
fun PastRidesScreen(onBack: () -> Unit, onShowTaps: (String) -> Unit, onShowRide: (String) -> Unit) {
    val recording by RecorderService.state.collectAsStateWithLifecycle()
    var rides by remember { mutableStateOf<List<PastRide>?>(null) }
    // Each ride's start and end from its data, filled in as they're read (each reads the ride's barometer).
    val spans = remember { mutableStateMapOf<String, SessionSpan>() }
    LaunchedEffect(Unit) {
        val list = withContext(Dispatchers.IO) { pastRides() }
        rides = list
        for (r in list) {
            withContext(Dispatchers.IO) { runCatching { sessionSpan(File(RecorderService.sessionsDir(), r.id).path) }.getOrNull() }
                ?.let { spans[r.id] = it }
        }
    }
    Column(
        Modifier.safeDrawingPadding().padding(24.dp).verticalScroll(rememberScrollState()),
        verticalArrangement = Arrangement.spacedBy(12.dp),
    ) {
        HapticButton(onClick = onBack) { Text("Back") }
        Text("Past rides", style = MaterialTheme.typography.headlineMedium)
        Text(
            "Kept in RideLogger/sessions on the phone; copy them off with `just pull`.",
            style = MaterialTheme.typography.bodySmall,
        )
        val list = rides
        when {
            list == null -> Text("Loading…")
            list.isEmpty() -> Text("No recordings yet.")
            else -> list.forEach { r ->
                val span = spans[r.id]
                Column {
                    Row(verticalAlignment = Alignment.CenterVertically, horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                        Text(
                            span?.let(::describeSpan) ?: r.id,
                            Modifier.weight(1f),
                            style = MaterialTheme.typography.titleMedium,
                        )
                        // The ride being recorded is still being written.
                        if (r.id != recording.sessionId) HapticOutlinedButton(onClick = { onShowRide(r.id) }) { Text("Ride") }
                        HapticOutlinedButton(onClick = { onShowTaps(r.id) }) { Text("Taps") }
                    }
                    Text(
                        listOfNotNull(
                            "%.1f MB".format(r.bytes / 1e6),
                            if (r.recompressed) "compacted" else null,
                            r.id,
                        ).joinToString(" · "),
                        style = MaterialTheme.typography.bodySmall,
                    )
                }
            }
        }
    }
}

/**
 * A past ride's map and graph, from an offline replay of the whole ride (`replay_session` in `crates/ffi`),
 * smoothed over all of it, not the live estimates from recording. Cached until the Rust core changes.
 * While recording, a cached replay is shown as usual, but an uncached ride is only replayed on request,
 * and only if it's at most [MAX_REPLAY_WHILE_RECORDING_S] long: a replay's memory grows with the ride.
 */
/**
 * The longest ride (s) to offer to replay while recording: 1:00:00. A replay's peak memory on the laptop grows
 * with the ride, about 230 MB on a typical 45 min one and 0.85 GB on the 3:14:27 one (since `18d424b`),
 * so an hour stays near 300 MB.
 */
private const val MAX_REPLAY_WHILE_RECORDING_S = 3600L

@Composable
fun PastRideScreen(id: String, onBack: () -> Unit) {
    val context = LocalContext.current
    val state by RecorderService.state.collectAsStateWithLifecycle()
    var replay by remember { mutableStateOf<RideReplay?>(null) }
    var error by remember { mutableStateOf<String?>(null) }
    var showGraph by remember { mutableStateOf(false) }
    // The ride's taps (not deleted ones), shown on the graph and map with the "Taps" button.
    var showTaps by remember { mutableStateOf(false) }
    var taps by remember { mutableStateOf<List<TapMark>>(emptyList()) }
    LaunchedEffect(id) {
        taps = withContext(Dispatchers.IO) {
            runCatching { sessionMarks(File(RecorderService.sessionsDir(), id).path) }
                .onFailure { Log.e("RideLogger", "reading taps failed", it) }
                .getOrDefault(emptyList())
        }.filter { !it.deleted }.map { m ->
            val kind = ManualEvent.Kind.forNumber(m.kind) ?: ManualEvent.Kind.UNRECOGNIZED
            TapMark(m.receivedNs, m.unixMs, kindLabel(kind) + if (m.text.isNotEmpty()) ": ${m.text}" else "")
        }
    }
    // How far the replay has got, once the session is read, and when that was (elapsed realtime ms).
    var progress by remember { mutableStateOf<Double?>(null) }
    var progressStartMs by remember { mutableStateOf(0L) }
    // While recording: whether there's no cached replay, the ride's length (s) once read, and whether the rider asked to replay it.
    var uncached by remember { mutableStateOf(false) }
    var durationS by remember { mutableStateOf<Long?>(null) }
    var replayAnyway by remember { mutableStateOf(false) }
    val replaysDir = File(context.filesDir, "replays").path
    LaunchedEffect(id, state.recording, replayAnyway) {
        if (replay != null) return@LaunchedEffect
        if (state.recording && !replayAnyway) {
            val sessionDir = File(RecorderService.sessionsDir(), id).path
            val cached = withContext(Dispatchers.IO) { runCatching { cachedReplay(sessionDir, replaysDir) }.getOrNull() }
            if (cached != null) {
                replay = cached
                return@LaunchedEffect
            }
            uncached = true
            durationS = withContext(Dispatchers.IO) {
                runCatching { sessionSpan(sessionDir) }.getOrNull()
                    ?.let { s -> s.endUnixMs?.let { (it - s.startUnixMs) / 1000 } }
            }
            return@LaunchedEffect
        }
        val listener = object : ReplayProgress {
            override fun onProgress(fraction: Double) {
                if (fraction == 0.0) progressStartMs = SystemClock.elapsedRealtime()
                progress = fraction
            }
        }
        withContext(Dispatchers.Default) {
            runCatching {
                replaySession(
                    File(RecorderService.sessionsDir(), id).path,
                    replaysDir,
                    listener,
                )
            }
        }.onSuccess { replay = it }.onFailure {
            Log.e("RideLogger", "ride replay failed", it)
            error = it.toString()
        }
    }
    val r = replay
    if (r == null) {
        Column(Modifier.safeDrawingPadding().padding(24.dp), verticalArrangement = Arrangement.spacedBy(12.dp)) {
            HapticButton(onClick = onBack) { Text("Back") }
            val p = progress
            Text(
                when {
                    error != null -> "The replay failed: $error"
                    state.recording && !replayAnyway -> when {
                        !uncached -> "Looking for this ride's saved replay…"
                        durationS.let { it != null && it <= MAX_REPLAY_WHILE_RECORDING_S } ->
                            "This ride hasn't been replayed yet. It's short enough (${clockDuration(durationS!!.toDouble())}) " +
                                "to replay while recording, but it takes memory the recording also needs."
                        else -> "Stop recording to replay this ride: it's over " +
                            "${clockDuration(MAX_REPLAY_WHILE_RECORDING_S.toDouble())} long, and its replay could cost the recording memory."
                    }
                    p == null -> "Reading the ride…"
                    p >= 1.0 -> "Smoothing the whole ride…"
                    // From the pace so far, once there's enough of it to go by.
                    p >= 0.03 -> {
                        val elapsedS = (SystemClock.elapsedRealtime() - progressStartMs) / 1000.0
                        "Replaying the ride: %.0f%%, about %s left".format(p * 100, clockDuration(elapsedS / p * (1 - p)))
                    }
                    else -> "Replaying the ride: %.0f%%".format(p * 100)
                },
            )
            if (state.recording && !replayAnyway && uncached && durationS.let { it != null && it <= MAX_REPLAY_WHILE_RECORDING_S }) {
                HapticOutlinedButton(onClick = { replayAnyway = true }) { Text("Replay now") }
            }
            if (error == null && (!state.recording || replayAnyway)) {
                if (p == null || p >= 1.0) {
                    LinearProgressIndicator(Modifier.fillMaxWidth())
                } else {
                    LinearProgressIndicator(progress = { p.toFloat() }, modifier = Modifier.fillMaxWidth())
                }
            }
        }
        return
    }
    // Fixed widths, so the button after one never moves when its text changes.
    val toggle = @Composable {
        HapticButton(onClick = { showGraph = !showGraph }, Modifier.width(96.dp)) { Text(if (showGraph) "Map" else "Graph") }
        HapticButton(onClick = { showTaps = !showTaps }, Modifier.width(112.dp)) { Text(if (showTaps) "Taps: on" else "Taps: off") }
    }
    val shownTaps = if (showTaps) taps else emptyList()
    if (showGraph) {
        val samples = remember(r) { r.smoothed.map { LiveSeries.Sample(it.elapsedRealtimeNs, it.speedMps, it.accelMps2) } }
        SpeedGraphScreen(samples, r.stops, r.timeline, pastClockOffsetMs = r.clockOffsetMs, onBack, toggle, taps = shownTaps)
    } else {
        val fixes = remember(r) {
            r.track.map {
                LiveTrack.Point(
                    it.elapsedRealtimeNs, it.timeMs, it.lat, it.lon,
                    fusedSpeedMps = null,
                    gpsSpeedMps = it.gpsSpeedMps,
                    gpsSpeedAccuracyMps = it.gpsSpeedAccuracyMps,
                    accuracyM = it.accuracyM,
                    startsSegment = it.startsSegment,
                    smoothedSpeedMps = it.speedMps,
                    smoothedAccelMps2 = it.accelMps2,
                )
            }
        }
        // The fused path (Rust's `Replay::map_path`, joined to the subway track underground), with the GPS fixes
        // over it; a ride with no fused path shows its fixes alone.
        val points = remember(r) {
            r.positions.map {
                LiveTrack.Point(
                    it.elapsedRealtimeNs, r.clockOffsetMs + it.elapsedRealtimeNs / 1_000_000, it.lat, it.lon,
                    fusedSpeedMps = null,
                    gpsSpeedMps = null,
                    gpsSpeedAccuracyMps = null,
                    accuracyM = it.sigmaM,
                    startsSegment = false,
                    smoothedSpeedMps = it.speedMps,
                    smoothedAccelMps2 = it.accelMps2,
                )
            }
        }
        // On the fused path, or the fixes if there's none; from Rust (`positions_at`), once per ride.
        val placed = remember(r, taps) {
            runCatching { placeTaps(taps, points.ifEmpty { fixes }) }
                .onFailure { Log.e("RideLogger", "placing taps failed", it) }
                .getOrDefault(emptyList())
        }
        val shownPlaced = if (showTaps) placed else emptyList()
        if (points.isEmpty()) {
            TrackMapScreen(fixes, live = false, onBack, toggle, taps = shownPlaced)
        } else {
            TrackMapScreen(points, live = false, onBack, toggle, gpsFixes = fixes, taps = shownPlaced)
        }
    }
}
