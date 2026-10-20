package dev.khyber.ride

import android.content.Context
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Switch
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.unit.dp

/**
 * Which units the app shows speeds, accelerations, and distances in: display only, since recordings and
 * everything computed from them stay in SI units. Each is a separate preference, kept across restarts.
 */
object Units {
    private const val PREFS = "units"
    private const val MPH = "mph"
    private const val FEET_PER_S2 = "ft_per_s2"
    private const val FEET = "feet"
    private const val MPS_PER_MPH = 0.44704
    private const val M_PER_FT = 0.3048

    var mph by mutableStateOf(false)
        private set
    var feetPerS2 by mutableStateOf(false)
        private set
    var feet by mutableStateOf(false)
        private set

    fun load(context: Context) {
        val p = context.getSharedPreferences(PREFS, Context.MODE_PRIVATE)
        mph = p.getBoolean(MPH, false)
        feetPerS2 = p.getBoolean(FEET_PER_S2, false)
        feet = p.getBoolean(FEET, false)
    }

    private fun save(context: Context, key: String, value: Boolean) {
        context.getSharedPreferences(PREFS, Context.MODE_PRIVATE).edit().putBoolean(key, value).apply()
    }

    val speedUnit get() = if (mph) "mph" else "km/h"
    val accelUnit get() = if (feetPerS2) "ft/s²" else "m/s²"
    val distanceUnit get() = if (feet) "ft" else "m"

    /** A speed (m/s) in the chosen unit. */
    fun speed(mps: Double) = if (mph) mps / MPS_PER_MPH else mps * 3.6

    /** A band boundary (km/h, as the color scales define them) in the chosen unit. */
    fun speedFromKmh(kmh: Double) = speed(kmh / 3.6)

    fun accel(mps2: Double) = if (feetPerS2) mps2 / M_PER_FT else mps2

    fun distance(m: Double) = if (feet) m / M_PER_FT else m

    /** Toggles for each, for the main screen. */
    @Composable
    fun Settings() {
        val context = LocalContext.current
        Column(verticalArrangement = Arrangement.spacedBy(4.dp)) {
            Toggle("Speed in mph", mph) { mph = it; save(context, MPH, it) }
            Toggle("Acceleration in ft/s²", feetPerS2) { feetPerS2 = it; save(context, FEET_PER_S2, it) }
            Toggle("Distances in feet", feet) { feet = it; save(context, FEET, it) }
        }
    }

    @Composable
    private fun Toggle(label: String, checked: Boolean, onChange: (Boolean) -> Unit) {
        Row(verticalAlignment = Alignment.CenterVertically, horizontalArrangement = Arrangement.spacedBy(12.dp)) {
            Switch(checked = checked, onCheckedChange = onChange)
            Text(label, style = MaterialTheme.typography.bodyMedium)
        }
    }
}
