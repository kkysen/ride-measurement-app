package dev.khyber.ride

import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.safeDrawingPadding
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.verticalScroll
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
import androidx.compose.ui.unit.dp
import java.io.File
import java.time.Instant
import java.time.ZoneId
import java.time.format.DateTimeFormatter
import java.util.Locale
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.withContext
import uniffi.ride_ffi.SessionSpan
import uniffi.ride_ffi.sessionSpan

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
fun PastRidesScreen(onBack: () -> Unit, onShowTaps: (String) -> Unit) {
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
