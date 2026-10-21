package dev.khyber.ride

import android.bluetooth.le.ScanSettings
import android.content.Context
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.FlowRow
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.setValue
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.unit.dp

/**
 * How often the radios are polled while recording: more often times stations better, less often saves battery.
 * Kept across restarts, and applied at once, mid-ride too.
 */
object Scanning {
    enum class Level(
        val label: String,
        val cellPollMs: Long,
        /** How often to request a Wi-Fi scan; null: never (other apps' scans are still recorded). */
        val wifiScanIntervalNs: Long?,
        val bleScanMode: Int,
    ) {
        FREQUENT("Frequent", 1_000L, 4_000_000_000L, ScanSettings.SCAN_MODE_LOW_LATENCY),
        NORMAL("Normal", 5_000L, 30_000_000_000L, ScanSettings.SCAN_MODE_BALANCED),
        SAVER("Battery saver", 15_000L, null, ScanSettings.SCAN_MODE_LOW_POWER),
    }

    private const val PREFS = "scanning"
    private const val LEVEL = "level"

    var level by mutableStateOf(Level.FREQUENT)
        private set

    /** Called after the level changes, e.g. to restart BLE scans with the new mode. */
    @Volatile
    var onChange: (() -> Unit)? = null

    fun load(context: Context) {
        val name = context.getSharedPreferences(PREFS, Context.MODE_PRIVATE).getString(LEVEL, null)
        level = Level.entries.firstOrNull { it.name == name } ?: Level.FREQUENT
    }

    private fun set(context: Context, l: Level) {
        if (l == level) return
        level = l
        context.getSharedPreferences(PREFS, Context.MODE_PRIVATE).edit().putString(LEVEL, l.name).apply()
        Health.guard("scanning level") { onChange?.invoke() }
    }

    /** One button per level, the chosen one filled, for the main screen. */
    @Composable
    fun Settings() {
        val context = LocalContext.current
        Column(verticalArrangement = Arrangement.spacedBy(4.dp)) {
            Text("Radio scanning (cell, Wi-Fi, Bluetooth)", style = MaterialTheme.typography.bodyMedium)
            FlowRow(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                for (l in Level.entries) {
                    if (l == level) {
                        HapticButton(onClick = {}) { Text(l.label) }
                    } else {
                        HapticOutlinedButton(onClick = { set(context, l) }) { Text(l.label) }
                    }
                }
            }
        }
    }
}
