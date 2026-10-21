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
    // Stored under their earlier names (ft/s², feet), so the choice carries over.
    private const val MPH_PER_S = "ft_per_s2"
    private const val IMPERIAL_DISTANCE = "feet"
    private const val MPS_PER_MPH = 0.44704
    private const val M_PER_FT = 0.3048
    private const val M_PER_MI = 1609.344

    var mph by mutableStateOf(false)
        private set
    var mphPerS by mutableStateOf(false)
        private set
    var imperialDistance by mutableStateOf(false)
        private set

    fun load(context: Context) {
        val p = context.getSharedPreferences(PREFS, Context.MODE_PRIVATE)
        mph = p.getBoolean(MPH, false)
        mphPerS = p.getBoolean(MPH_PER_S, false)
        imperialDistance = p.getBoolean(IMPERIAL_DISTANCE, false)
    }

    private fun save(context: Context, key: String, value: Boolean) {
        context.getSharedPreferences(PREFS, Context.MODE_PRIVATE).edit().putBoolean(key, value).apply()
    }

    val speedUnit get() = if (mph) "mph" else "km/h"
    val accelUnit get() = if (mphPerS) "mph/s" else "m/s²"

    /** A speed (m/s) in the chosen unit. */
    fun speed(mps: Double) = if (mph) mps / MPS_PER_MPH else mps * 3.6

    /** A band boundary (km/h, as the color scales define them) in the chosen unit. */
    fun speedFromKmh(kmh: Double) = speed(kmh / 3.6)

    fun accel(mps2: Double) = if (mphPerS) mps2 / MPS_PER_MPH else mps2

    /** A distance (m) with its unit, in whichever fits its size: m or km, or ft or mi. */
    fun distance(m: Double): String = when {
        imperialDistance && m < 1000 * M_PER_FT -> "%.0f ft".format(m / M_PER_FT)
        imperialDistance -> "%.1f mi".format(m / M_PER_MI)
        m < 1000 -> "%.0f m".format(m)
        else -> "%.1f km".format(m / 1000)
    }

    /** Toggles for each, for the main screen. */
    @Composable
    fun Settings() {
        val context = LocalContext.current
        Column(verticalArrangement = Arrangement.spacedBy(4.dp)) {
            Toggle("Speed in mph", mph) { mph = it; save(context, MPH, it) }
            Toggle("Acceleration in mph/s", mphPerS) { mphPerS = it; save(context, MPH_PER_S, it) }
            Toggle("Distances in feet and miles", imperialDistance) { imperialDistance = it; save(context, IMPERIAL_DISTANCE, it) }
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
