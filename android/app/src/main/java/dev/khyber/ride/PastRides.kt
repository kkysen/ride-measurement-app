package dev.khyber.ride

import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.verticalScroll
import androidx.compose.material3.Button
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Modifier
import androidx.compose.ui.unit.dp
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.withContext
import java.io.File
import java.text.SimpleDateFormat
import java.util.Locale

/** One recording on the phone, as far as its files tell without decoding them. */
private data class PastRide(val id: String, val start: Long?, val lastWrite: Long, val bytes: Long, val recompressed: Boolean)

private val ID_FORMAT = SimpleDateFormat("yyyyMMdd-HHmmss", Locale.US)
private val SHOWN_FORMAT = SimpleDateFormat("EEE d MMM, HH:mm", Locale.getDefault())

private fun pastRides(): List<PastRide> =
    RecorderService.sessionsDir().listFiles().orEmpty().filter { it.isDirectory }.map { dir ->
        val files = dir.walkTopDown().filter { it.isFile }.toList()
        PastRide(
            id = dir.name,
            // Session IDs are their start time (with a suffix if two start in the same second).
            start = runCatching { ID_FORMAT.parse(dir.name.take(15))?.time }.getOrNull(),
            lastWrite = files.maxOfOrNull { it.lastModified() } ?: dir.lastModified(),
            bytes = files.sumOf { it.length() },
            recompressed = File(dir, ".recompressed").exists(),
        )
    }.sortedByDescending { it.id }

/** The recordings on the phone, newest first: when, about how long, and how big. */
@Composable
fun PastRidesScreen(onBack: () -> Unit) {
    var rides by remember { mutableStateOf<List<PastRide>?>(null) }
    LaunchedEffect(Unit) { rides = withContext(Dispatchers.IO) { pastRides() } }
    Column(
        Modifier.padding(24.dp).verticalScroll(rememberScrollState()),
        verticalArrangement = Arrangement.spacedBy(12.dp),
    ) {
        Button(onClick = onBack) { Text("Back") }
        Text("Past rides", style = MaterialTheme.typography.headlineMedium)
        Text(
            "Kept in RideLogger/sessions on the phone; copy them off with `just pull`. " +
                "Lengths are approximate, from when files were last written.",
            style = MaterialTheme.typography.bodySmall,
        )
        val list = rides
        when {
            list == null -> Text("Loading…")
            list.isEmpty() -> Text("No recordings yet.")
            else -> list.forEach { r ->
                val minutes = r.start?.let { (r.lastWrite - it) / 60_000.0 }
                Column {
                    Text(r.start?.let { SHOWN_FORMAT.format(it) } ?: r.id, style = MaterialTheme.typography.titleMedium)
                    Text(
                        listOfNotNull(
                            minutes?.let { if (it < 1) "under a minute" else "about %.0f min".format(it) },
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
