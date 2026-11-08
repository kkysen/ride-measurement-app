package dev.khyber.ride

import android.os.SystemClock
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.FlowRow
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.safeDrawingPadding
import androidx.compose.foundation.lazy.LazyColumn
import androidx.compose.foundation.lazy.items
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableIntStateOf
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.rememberCoroutineScope
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.text.style.TextDecoration
import androidx.compose.ui.unit.dp
import dev.khyber.ride.proto.ManualEvent
import java.io.File
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.launch
import kotlinx.coroutines.withContext
import uniffi.ride_ffi.SessionMark
import uniffi.ride_ffi.editSessionMark
import uniffi.ride_ffi.sessionMarks

/** The kinds a tap can be changed to: every button's. */
private val KINDS = ManualEvent.Kind.entries.filter {
    it != ManualEvent.Kind.KIND_UNSPECIFIED && it != ManualEvent.Kind.KIND_DELETE && it != ManualEvent.Kind.UNRECOGNIZED
}

fun kindLabel(kind: ManualEvent.Kind): String = kind.name.removePrefix("KIND_").lowercase().replace('_', ' ')

/**
 * A finished ride's taps, in time order, each deletable or changeable to another kind.
 * Rust reads and edits them (`ride_session::marks`), appending records, so nothing is ever lost.
 */
@Composable
fun RideTapsScreen(sessionId: String, onBack: () -> Unit) {
    val dir = File(RecorderService.sessionsDir(), sessionId).path
    var marks by remember { mutableStateOf<List<SessionMark>?>(null) }
    var error by remember { mutableStateOf<String?>(null) }
    var reload by remember { mutableIntStateOf(0) }
    // The tap whose kinds are showing, and the one waiting for its delete to be confirmed.
    var changing by remember { mutableStateOf<Long?>(null) }
    var confirming by remember { mutableStateOf<Long?>(null) }
    val scope = rememberCoroutineScope()
    LaunchedEffect(reload) {
        withContext(Dispatchers.IO) {
            try {
                marks = sessionMarks(dir)
            } catch (e: Exception) {
                error = e.message
            }
        }
    }
    fun edit(mark: SessionMark, kind: ManualEvent.Kind?) {
        changing = null
        confirming = null
        scope.launch(Dispatchers.IO) {
            try {
                editSessionMark(dir, mark.receivedNs, kind?.number, mark.text, SystemClock.elapsedRealtimeNanos(), System.currentTimeMillis())
            } catch (e: Exception) {
                error = e.message
            }
            reload++
        }
    }
    Column(
        Modifier.fillMaxSize().safeDrawingPadding().padding(16.dp),
        verticalArrangement = Arrangement.spacedBy(8.dp),
    ) {
        HapticButton(onClick = onBack) { Text("Back") }
        Text("Taps in $sessionId", style = MaterialTheme.typography.titleMedium)
        error?.let { Text(it, color = MaterialTheme.colorScheme.error) }
        val list = marks
        when {
            list == null -> Text("Loading…")
            list.isEmpty() -> Text("No taps in this ride.")
        }
        LazyColumn(verticalArrangement = Arrangement.spacedBy(6.dp)) {
            items(list.orEmpty(), key = { it.receivedNs }) { mark ->
                val kind = ManualEvent.Kind.forNumber(mark.kind) ?: ManualEvent.Kind.UNRECOGNIZED
                Column {
                    Row(verticalAlignment = Alignment.CenterVertically, horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                        Text(
                            "${clockTime(mark.unixMs)}  ${kindLabel(kind)}" +
                                (if (mark.text.isNotEmpty()) ": ${mark.text}" else "") +
                                (if (mark.edited) " (changed)" else ""),
                            Modifier.weight(1f),
                            style = MaterialTheme.typography.bodyLarge,
                            textDecoration = if (mark.deleted) TextDecoration.LineThrough else null,
                        )
                        when {
                            mark.deleted -> Text("deleted", style = MaterialTheme.typography.bodySmall)
                            confirming == mark.receivedNs -> HapticButton(onClick = { edit(mark, null) }) { Text("Confirm") }
                            else -> {
                                HapticOutlinedButton(onClick = {
                                    changing = if (changing == mark.receivedNs) null else mark.receivedNs
                                }) { Text("Change") }
                                HapticOutlinedButton(onClick = { confirming = mark.receivedNs }) { Text("Delete") }
                            }
                        }
                    }
                    if (changing == mark.receivedNs) {
                        FlowRow(horizontalArrangement = Arrangement.spacedBy(6.dp)) {
                            KINDS.filter { it != kind }.forEach { k ->
                                HapticOutlinedButton(onClick = { edit(mark, k) }) { Text(kindLabel(k)) }
                            }
                        }
                    }
                }
            }
        }
    }
}
