package dev.khyber.ride

import android.os.SystemClock
import androidx.compose.foundation.Canvas
import androidx.compose.foundation.gestures.detectTapGestures
import androidx.compose.foundation.gestures.detectTransformGestures
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
import androidx.compose.runtime.mutableFloatStateOf
import androidx.compose.runtime.mutableIntStateOf
import androidx.compose.runtime.mutableLongStateOf
import androidx.compose.runtime.rememberUpdatedState
import androidx.compose.runtime.saveable.rememberSaveable
import androidx.compose.runtime.setValue
import androidx.compose.ui.Modifier
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.geometry.Size
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.Path
import androidx.compose.ui.graphics.PathEffect
import androidx.compose.ui.graphics.drawscope.DrawScope
import androidx.compose.ui.graphics.drawscope.Stroke
import androidx.compose.ui.input.pointer.pointerInput
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

/** The narrowest the zoomed graph gets (ns). */
private const val MIN_SPAN_NS = 30_000_000_000L

/**
 * The current recording's speed and acceleration as line graphs over time, one above the other,
 * with each train stop shaded from its stop to its departure (the doors' opening darker),
 * labeled with its station, dwell, and how long the doors were open.
 * Pinching zooms in time only, dragging pans; a double tap shows the whole window again.
 */
@Composable
fun LiveGraphScreen(onBack: () -> Unit) {
    val samples by LiveSeries.samples.collectAsStateWithLifecycle()
    val stops by LiveSeries.stops.collectAsStateWithLifecycle()
    var window by rememberSaveable { mutableIntStateOf(1) }
    // The zoom (1 shows the whole window), and how far before the window's end the view ends (ns).
    var zoom by rememberSaveable { mutableFloatStateOf(1f) }
    var backNs by rememberSaveable { mutableLongStateOf(0L) }
    val text = rememberTextMeasurer()
    val ink = MaterialTheme.colorScheme.onSurface
    val speedColor = MaterialTheme.colorScheme.primary
    val accelColor = MaterialTheme.colorScheme.tertiary

    Column(Modifier.fillMaxSize().safeDrawingPadding().padding(16.dp), verticalArrangement = Arrangement.spacedBy(8.dp)) {
        Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
            HapticButton(onClick = onBack) { Text("Back") }
            HapticButton(onClick = {
                window = (window + 1) % WINDOWS_MIN.size
                zoom = 1f
                backNs = 0L
            }) {
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
        val windowEndNs = max(samples.last().elapsedRealtimeNs, nowNs)
        val windowStartNs = WINDOWS_MIN[window]?.let { max(windowEndNs - it * 60_000_000_000L, samples.first().elapsedRealtimeNs) }
            ?: samples.first().elapsedRealtimeNs
        val windowSpanNs = max(1L, windowEndNs - windowStartNs)
        val spanNs = max(min(MIN_SPAN_NS, windowSpanNs), (windowSpanNs / zoom).toLong())
        // Kept in the window; at its end, the view follows now.
        val endNs = windowEndNs - backNs.coerceIn(0L, windowSpanNs - spanNs)
        val startNs = endNs - spanNs
        val shown = samples.filter { it.elapsedRealtimeNs in startNs..endNs }
        val shownStops = stops.filter { (it.departedNs ?: it.leftNs ?: nowNs) >= startNs && it.stoppedNs <= endNs }
        val clock = { ns: Long -> CLOCK.format(Instant.ofEpochMilli(clockOffsetMs + ns / 1_000_000)).lowercase() }
        // The gestures outlive each frame's range (now moves on), so they read the latest one.
        val view by rememberUpdatedState(longArrayOf(windowEndNs, windowSpanNs, startNs, spanNs))
        val gestures = Modifier
            .pointerInput(Unit) {
                detectTransformGestures { centroid, pan, zoomChange, _ ->
                    val (windowEndNs, windowSpanNs, startNs, spanNs) = view
                    val width = size.width.toFloat()
                    // The time under the fingers stays under them.
                    val anchor = (centroid.x / width).coerceIn(0f, 1f)
                    val newZoom = (zoom * zoomChange).coerceIn(1f, max(1f, windowSpanNs.toFloat() / MIN_SPAN_NS))
                    val newSpan = (windowSpanNs / newZoom).toLong()
                    val anchorNs = startNs + (spanNs * anchor).toLong()
                    val newEnd = anchorNs + (newSpan * (1 - anchor)).toLong() - (pan.x / width * newSpan).toLong()
                    zoom = newZoom
                    backNs = (windowEndNs - newEnd).coerceIn(0L, windowSpanNs - newSpan)
                }
            }
            .pointerInput(Unit) {
                detectTapGestures(onDoubleTap = {
                    zoom = 1f
                    backNs = 0L
                })
            }

        Text("Speed (${Units.speedUnit})", color = speedColor)
        Canvas(Modifier.fillMaxWidth().weight(1f).then(gestures)) {
            plot(text, ink, speedColor, shown.map { it.elapsedRealtimeNs to Units.speed(it.speedMps) }, startNs, endNs, shownStops, nowNs, zeroBased = true, labels = true)
        }
        Text("Acceleration (${Units.accelUnit}), negative is braking", color = accelColor)
        Canvas(Modifier.fillMaxWidth().weight(1f).then(gestures)) {
            plot(text, ink, accelColor, shown.map { it.elapsedRealtimeNs to Units.accel(it.accelMps2) }, startNs, endNs, shownStops, nowNs, zeroBased = false, labels = false)
        }
        Row(Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.SpaceBetween) {
            Text(clock(startNs))
            Text(clock(endNs))
        }
    }
}

/**
 * One line graph: gridlines with their values on the left, the stops as shaded spans, and the line.
 * With `labels`, each stop is named above its span with its dwell and doors' time, where there's room.
 */
private fun DrawScope.plot(
    text: TextMeasurer,
    ink: Color,
    color: Color,
    points: List<Pair<Long, Double>>,
    startNs: Long,
    endNs: Long,
    stops: List<LiveStop>,
    nowNs: Long,
    zeroBased: Boolean,
    labels: Boolean,
) {
    if (points.isEmpty()) return
    val labelStyle = TextStyle(color = ink, fontSize = 11.sp)
    val left = 36.dp.toPx()
    val top = (if (labels) 30 else 4).dp.toPx()
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
    var labelsEnd = left
    for (s in stops) {
        // Standing until the departure, or the rider walking off, or now.
        val endNs = s.departedNs ?: s.leftNs ?: nowNs
        val sx = x(s.stoppedNs).coerceAtLeast(left)
        val ex = x(endNs).coerceAtMost(size.width)
        drawRect(ink.copy(alpha = 0.08f), Offset(sx, top), Size(max(1f, ex - sx), h))
        val open = s.doorsOpenNs
        if (open != null) {
            val ox = x(open).coerceIn(left, size.width)
            val cx = x(s.doorsClosedNs ?: endNs).coerceIn(left, size.width)
            drawRect(ink.copy(alpha = 0.12f), Offset(ox, top), Size(max(1f, cx - ox), h))
        }
        drawLine(ink.copy(alpha = 0.6f), Offset(sx, top), Offset(sx, size.height), 1.5f, pathEffect = dash)
        if (!labels) continue
        val dwell = "dwell " + clockDuration((endNs - s.stoppedNs) / 1e9) +
            (s.doorsClosedNs?.let { c -> open?.let { " · doors " + clockDuration((c - it) / 1e9) } } ?: "")
        val name = text.measure(s.station ?: "stop", labelStyle)
        val times = text.measure(dwell, labelStyle)
        val width = max(name.size.width, times.size.width)
        // Only where it won't overlap the last label or run off the right.
        if (sx >= labelsEnd && sx + width <= size.width) {
            drawText(name, topLeft = Offset(sx, 0f))
            drawText(times, topLeft = Offset(sx, name.size.height.toFloat()))
            labelsEnd = sx + width + 4.dp.toPx()
        }
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
