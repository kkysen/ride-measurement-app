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
import androidx.compose.foundation.layout.height
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
import androidx.compose.ui.unit.Constraints
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
import uniffi.ride_ffi.TimelineSegment
import uniffi.ride_ffi.clockTicks

/** How much of the ride the graph shows, ending now; null is the whole ride. */
private val WINDOWS_MIN = listOf(5, 15, 60, null)
private val NEW_YORK = ZoneId.of("America/New_York")
private val CLOCK = DateTimeFormatter.ofPattern("h:mm a", java.util.Locale.US).withZone(NEW_YORK)

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
    val timeline by LiveSeries.timeline.collectAsStateWithLifecycle()
    SpeedGraphScreen(samples, stops, timeline, pastClockOffsetMs = null, onBack)
}

/**
 * The graphs for the current recording, or for a past ride if [pastClockOffsetMs] (unix ms minus elapsed-realtime ms
 * over that ride) is given: then the ride's end stands for now, and it's shown whole at first.
 */
@Composable
fun SpeedGraphScreen(
    samples: List<LiveSeries.Sample>,
    stops: List<LiveStop>,
    timeline: List<TimelineSegment>,
    pastClockOffsetMs: Long?,
    onBack: () -> Unit,
    extraButton: @Composable () -> Unit = {},
) {
    var window by rememberSaveable { mutableIntStateOf(if (pastClockOffsetMs == null) 1 else WINDOWS_MIN.lastIndex) }
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
            extraButton()
            HapticButton(onClick = {
                window = (window + 1) % WINDOWS_MIN.size
                zoom = 1f
                backNs = 0L
            }) {
                Text(WINDOWS_MIN[window]?.let { if (it < 60) "Last $it min" else "Last ${it / 60} h" } ?: "Whole ride")
            }
        }
        if (samples.size < 2) {
            Text(if (pastClockOffsetMs == null) "No speeds yet: they start once a recording does." else "No speeds on this ride.")
            return@Column
        }
        // Elapsed-realtime times, mapped to the clock by the offset now, or the past ride's.
        val nowNs = if (pastClockOffsetMs == null) SystemClock.elapsedRealtimeNanos() else samples.last().elapsedRealtimeNs
        val clockOffsetMs = pastClockOffsetMs ?: (System.currentTimeMillis() - nowNs / 1_000_000)
        val windowEndNs = max(samples.last().elapsedRealtimeNs, nowNs)
        val windowStartNs = WINDOWS_MIN[window]?.let { max(windowEndNs - it * 60_000_000_000L, samples.first().elapsedRealtimeNs) }
            ?: samples.first().elapsedRealtimeNs
        val windowSpanNs = max(1L, windowEndNs - windowStartNs)
        // Divided in doubles and capped: a `Float` division rounds an hour in ns past the window, emptying the pan range.
        val spanNs = min(windowSpanNs, max(min(MIN_SPAN_NS, windowSpanNs), (windowSpanNs / zoom.toDouble()).toLong()))
        // Kept in the window; at its end, the view follows now.
        val endNs = windowEndNs - backNs.coerceIn(0L, windowSpanNs - spanNs)
        val startNs = endNs - spanNs
        val shown = samples.filter { it.elapsedRealtimeNs in startNs..endNs }
        val shownStops = stops.filter { (it.departedNs ?: it.leftNs ?: nowNs) >= startNs && it.stoppedNs <= endNs }
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
                    val newSpan = min(windowSpanNs, (windowSpanNs / newZoom.toDouble()).toLong())
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

        Canvas(Modifier.fillMaxWidth().height(20.dp).then(gestures)) {
            strip(text, ink, speedColor, timeline, startNs, endNs)
        }
        Text("Speed (${Units.speedUnit})", color = speedColor)
        Canvas(Modifier.fillMaxWidth().weight(1f).then(gestures)) {
            plot(text, ink, speedColor, shown.map { it.elapsedRealtimeNs to Units.speed(it.speedMps) }, startNs, endNs, shownStops, nowNs, zeroBased = true, labels = true)
        }
        Text("Acceleration (${Units.accelUnit}), negative is braking", color = accelColor)
        Canvas(Modifier.fillMaxWidth().weight(1f).then(gestures)) {
            plot(text, ink, accelColor, shown.map { it.elapsedRealtimeNs to Units.accel(it.accelMps2) }, startNs, endNs, shownStops, nowNs, zeroBased = false, labels = false)
        }
        Canvas(Modifier.fillMaxWidth().height(20.dp).then(gestures)) {
            timeAxis(text, ink, clockOffsetMs, startNs, endNs)
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
    // Room for a label: a name on two lines, then the dwell and the doors on a line each.
    val top = (if (labels) 58 else 4).dp.toPx()
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
    for ((i, s) in stops.withIndex()) {
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
        if (sx < labelsEnd) continue
        // A label gets the room up to the next stop's, and never overlaps another or runs off the right.
        val room = ((stops.getOrNull(i + 1)?.let { x(it.stoppedNs) } ?: size.width).coerceAtMost(size.width) - sx - 4.dp.toPx())
            .coerceAtMost(140.dp.toPx())
        if (room < 16.dp.toPx()) continue
        val constraints = Constraints(maxWidth = room.toInt())
        fun fits(line: String, maxLines: Int) =
            text.measure(line, labelStyle, maxLines = maxLines, constraints = constraints).takeIf { !it.hasVisualOverflow }
        val station = s.station ?: "stop"
        val name = fits(station, 2) ?: fits(shortName(station), 2) ?: continue
        val lines = listOfNotNull(
            name,
            fits("dwell " + clockDuration((endNs - s.stoppedNs) / 1e9), 1),
            s.doorsClosedNs?.let { c -> open?.let { fits("doors " + clockDuration((c - it) / 1e9), 1) } },
        )
        var y = 0f
        for (line in lines) {
            drawText(line, topLeft = Offset(sx, y))
            y += line.size.height
        }
        labelsEnd = sx + lines.maxOf { it.size.width } + 4.dp.toPx()
    }

    val path = Path()
    points.forEachIndexed { i, (t, v) ->
        if (i == 0) path.moveTo(x(t), y(v)) else path.lineTo(x(t), y(v))
    }
    drawPath(path, color, style = Stroke(width = 2.dp.toPx()))
}

/**
 * The time axis under the graphs: round clock times (12-hour, New York) from Rust's `clock_ticks`,
 * as many as fit, each centered on its time and kept on the canvas.
 */
private fun DrawScope.timeAxis(text: TextMeasurer, ink: Color, clockOffsetMs: Long, startNs: Long, endNs: Long) {
    val left = 36.dp.toPx()
    val w = size.width - left
    val startMs = clockOffsetMs + startNs / 1_000_000
    val endMs = clockOffsetMs + endNs / 1_000_000
    val style = TextStyle(color = ink, fontSize = 11.sp)
    val offsetMs = NEW_YORK.rules.getOffset(Instant.ofEpochMilli(startMs)).totalSeconds * 1000L
    val maxTicks = (w / 72.dp.toPx()).toInt().coerceAtLeast(2)
    for (ms in clockTicks(startMs, endMs, offsetMs, maxTicks.toUInt())) {
        val tx = left + w * (ms - startMs).toFloat() / max(1L, endMs - startMs).toFloat()
        val label = text.measure(CLOCK.format(Instant.ofEpochMilli(ms)).lowercase(), style)
        val lx = (tx - label.size.width / 2f).coerceIn(0f, max(0f, size.width - label.size.width))
        drawLine(ink.copy(alpha = 0.5f), Offset(tx, 0f), Offset(tx, 3.dp.toPx()), 1f)
        drawText(label, topLeft = Offset(lx, size.height - label.size.height))
    }
}

/** What the rider was doing, as a strip over the graphs' time axis: each line ridden, and walking, labeled where there's room. */
private fun DrawScope.strip(text: TextMeasurer, ink: Color, color: Color, segments: List<TimelineSegment>, startNs: Long, endNs: Long) {
    val left = 36.dp.toPx()
    val w = size.width - left
    val x = { ns: Long -> left + w * (ns - startNs).toFloat() / max(1L, endNs - startNs).toFloat() }
    val style = TextStyle(color = ink, fontSize = 11.sp)
    for (s in segments) {
        if (s.endNs < startNs || s.startNs > endNs) continue
        val sx = x(s.startNs).coerceAtLeast(left)
        val ex = x(s.endNs).coerceAtMost(size.width)
        val fill = if (s.walking) ink.copy(alpha = 0.15f) else color.copy(alpha = 0.3f)
        drawRect(fill, Offset(sx, 0f), Size(max(1f, ex - sx - 1f), size.height))
        val label = text.measure(s.label, style)
        if (label.size.width + 4.dp.toPx() <= ex - sx) {
            drawText(label, topLeft = Offset(sx + 2.dp.toPx(), (size.height - label.size.height) / 2))
        }
    }
}

/**
 * A station's name without its complex's other part: before the last dash that follows a letter
 * (`Jackson Hts-Roosevelt Av` is `Jackson Hts`, `47-50 Sts-Rockefeller Ctr` is `47-50 Sts`, `14 St-Union Sq` is `14 St`).
 */
private fun shortName(name: String): String {
    val i = name.indices.lastOrNull { name[it] == '-' && it > 0 && name[it - 1].isLetter() } ?: return name
    return name.substring(0, i)
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
