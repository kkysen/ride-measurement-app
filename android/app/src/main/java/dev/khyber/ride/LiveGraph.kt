package dev.khyber.ride

import android.os.SystemClock
import androidx.compose.foundation.Canvas
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.safeDrawingPadding
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableIntStateOf
import androidx.compose.runtime.saveable.rememberSaveable
import androidx.compose.runtime.setValue
import androidx.compose.ui.Modifier
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.Path
import androidx.compose.ui.graphics.PathEffect
import androidx.compose.ui.graphics.drawscope.DrawScope
import androidx.compose.ui.graphics.drawscope.Stroke
import androidx.compose.ui.text.TextMeasurer
import androidx.compose.ui.text.TextStyle
import androidx.compose.ui.text.drawText
import androidx.compose.ui.text.rememberTextMeasurer
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import androidx.lifecycle.compose.collectAsStateWithLifecycle
import java.time.Instant
import java.time.ZoneId
import java.time.format.DateTimeFormatter
import kotlin.math.ceil
import kotlin.math.floor
import kotlin.math.max
import kotlin.math.min
import uniffi.ride_ffi.LiveStop

/** How much of the ride the graph shows, ending now; null is the whole ride. */
private val WINDOWS_MIN = listOf(5, 15, 60, null)
private val CLOCK = DateTimeFormatter.ofPattern("h:mm a").withZone(ZoneId.of("America/New_York"))

/**
 * The current recording's speed and acceleration as line graphs over time, one above the other,
 * with each train stop marked by a dashed line at its stop and its station's name.
 */
@Composable
fun LiveGraphScreen(onBack: () -> Unit) {
    val samples by LiveSeries.samples.collectAsStateWithLifecycle()
    val stops by LiveSeries.stops.collectAsStateWithLifecycle()
    var window by rememberSaveable { mutableIntStateOf(1) }
    val text = rememberTextMeasurer()
    val ink = MaterialTheme.colorScheme.onSurface
    val speedColor = MaterialTheme.colorScheme.primary
    val accelColor = MaterialTheme.colorScheme.tertiary

    Column(Modifier.fillMaxSize().safeDrawingPadding().padding(16.dp), verticalArrangement = Arrangement.spacedBy(8.dp)) {
        Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
            HapticButton(onClick = onBack) { Text("Back") }
            HapticButton(onClick = { window = (window + 1) % WINDOWS_MIN.size }) {
                Text(WINDOWS_MIN[window]?.let { if (it < 60) "Last $it min" else "Last ${it / 60} h" } ?: "Whole ride")
            }
        }
        if (samples.size < 2) {
            Text("No speeds yet: they start once a recording does.")
            return@Column
        }
        // Elapsed-realtime times, mapped to the clock by the offset now.
        val nowNs = SystemClock.elapsedRealtimeNanos()
        val clockOffsetMs = System.currentTimeMillis() - nowNs / 1_000_000
        val endNs = max(samples.last().elapsedRealtimeNs, nowNs)
        val startNs = WINDOWS_MIN[window]?.let { max(endNs - it * 60_000_000_000L, samples.first().elapsedRealtimeNs) }
            ?: samples.first().elapsedRealtimeNs
        val shown = samples.filter { it.elapsedRealtimeNs >= startNs }
        val shownStops = stops.filter { it.stoppedNs in startNs..endNs }
        val clock = { ns: Long -> CLOCK.format(Instant.ofEpochMilli(clockOffsetMs + ns / 1_000_000)).lowercase() }

        Text("Speed (${Units.speedUnit})", color = speedColor)
        Canvas(Modifier.fillMaxWidth().weight(1f)) {
            plot(text, ink, speedColor, shown.map { it.elapsedRealtimeNs to Units.speed(it.speedMps) }, startNs, endNs, shownStops, zeroBased = true)
        }
        Text("Acceleration (${Units.accelUnit}), negative is braking", color = accelColor)
        Canvas(Modifier.fillMaxWidth().weight(1f)) {
            plot(text, ink, accelColor, shown.map { it.elapsedRealtimeNs to Units.accel(it.accelMps2) }, startNs, endNs, shownStops, zeroBased = false)
        }
        Row(Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.SpaceBetween) {
            Text(clock(startNs))
            Text(clock(endNs))
        }
    }
}

/** One line graph: gridlines with their values on the left, the line, and the stops as dashed lines. */
private fun DrawScope.plot(
    text: TextMeasurer,
    ink: Color,
    color: Color,
    points: List<Pair<Long, Double>>,
    startNs: Long,
    endNs: Long,
    stops: List<LiveStop>,
    zeroBased: Boolean,
) {
    if (points.isEmpty()) return
    val labelStyle = TextStyle(color = ink, fontSize = 11.sp)
    val left = 36.dp.toPx()
    val top = 16.dp.toPx()
    val w = size.width - left
    val h = size.height - top
    var lo = points.minOf { it.second }
    var hi = points.maxOf { it.second }
    if (zeroBased) lo = 0.0 else {
        // Symmetric about zero, so motoring and braking read alike.
        val m = max(-lo, hi).coerceAtLeast(0.5)
        lo = -m
        hi = m
    }
    if (hi - lo < 1.0) hi = lo + 1.0
    val step = niceStep((hi - lo) / 4)
    lo = floor(lo / step) * step
    hi = ceil(hi / step) * step
    val x = { ns: Long -> left + w * (ns - startNs).toFloat() / max(1L, endNs - startNs).toFloat() }
    val y = { v: Double -> top + h * (1 - ((v - lo) / (hi - lo)).toFloat()) }

    var g = lo
    while (g <= hi + step / 2) {
        val gy = y(g)
        drawLine(ink.copy(alpha = if (g == 0.0) 0.5f else 0.15f), Offset(left, gy), Offset(size.width, gy), 1f)
        val label = if (step >= 1) "%.0f".format(g) else "%.1f".format(g)
        drawText(text, label, Offset(0f, gy - 7.sp.toPx()), labelStyle)
        g += step
    }

    val dash = PathEffect.dashPathEffect(floatArrayOf(8f, 8f))
    for (s in stops) {
        val sx = x(s.stoppedNs)
        drawLine(ink.copy(alpha = 0.6f), Offset(sx, top), Offset(sx, size.height), 1.5f, pathEffect = dash)
        s.station?.let { drawText(text, it, Offset(min(sx + 2f, size.width - 60.dp.toPx()), 0f), labelStyle) }
    }

    val path = Path()
    points.forEachIndexed { i, (t, v) ->
        if (i == 0) path.moveTo(x(t), y(v)) else path.lineTo(x(t), y(v))
    }
    drawPath(path, color, style = Stroke(width = 2.dp.toPx()))
}

/** A round gridline spacing near `raw`: 1, 2, or 5 times a power of ten. */
private fun niceStep(raw: Double): Double {
    val p = Math.pow(10.0, floor(Math.log10(raw)))
    val f = raw / p
    return p * when {
        f <= 1 -> 1.0
        f <= 2 -> 2.0
        f <= 5 -> 5.0
        else -> 10.0
    }
}
