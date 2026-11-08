package dev.khyber.ride

import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.safeDrawingPadding
import androidx.compose.foundation.lazy.LazyColumn
import androidx.compose.foundation.lazy.items
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.text.style.TextDecoration
import androidx.compose.ui.unit.dp
import androidx.lifecycle.compose.collectAsStateWithLifecycle

/**
 * This session's taps, newest first, each deletable. Deleting records that the tap was wrong
 * (`KIND_DELETE`) and keeps the tap itself, so nothing is lost; it takes a second tap to confirm.
 */
@Composable
fun TapsScreen(onBack: () -> Unit) {
    val marks by RecorderService.marks.collectAsStateWithLifecycle()
    // The tap waiting for its delete to be confirmed, if any.
    var confirming by remember { mutableStateOf<Long?>(null) }
    Column(
        Modifier.fillMaxSize().safeDrawingPadding().padding(16.dp),
        verticalArrangement = Arrangement.spacedBy(8.dp),
    ) {
        HapticButton(onClick = onBack) { Text("Back") }
        if (marks.isEmpty()) Text("No taps in this session yet.")
        LazyColumn(verticalArrangement = Arrangement.spacedBy(4.dp)) {
            items(marks.asReversed(), key = { it.receivedNs }) { mark ->
                Row(verticalAlignment = Alignment.CenterVertically, horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                    val label = kindLabel(mark.kind) +
                        if (mark.text.isNotEmpty()) ": ${mark.text}" else ""
                    Text(
                        "${clockTime(mark.wallMs)}  $label",
                        Modifier.weight(1f),
                        style = MaterialTheme.typography.bodyLarge,
                        textDecoration = if (mark.deleted) TextDecoration.LineThrough else null,
                    )
                    when {
                        mark.deleted -> Text("deleted", style = MaterialTheme.typography.bodySmall)
                        confirming == mark.receivedNs -> HapticButton(onClick = {
                            RecorderService.deleteEvent(mark)
                            confirming = null
                        }) { Text("Confirm") }
                        else -> HapticOutlinedButton(onClick = { confirming = mark.receivedNs }) { Text("Delete") }
                    }
                }
            }
        }
    }
}
