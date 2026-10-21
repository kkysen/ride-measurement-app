package dev.khyber.ride

import android.annotation.SuppressLint
import android.content.Context
import android.graphics.Color
import android.graphics.DashPathEffect
import android.util.Log
import android.view.MotionEvent
import android.widget.TextView
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.padding
import androidx.compose.material3.Button
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.saveable.rememberSaveable
import androidx.compose.runtime.setValue
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clipToBounds
import androidx.compose.ui.unit.dp
import androidx.compose.ui.viewinterop.AndroidView
import androidx.lifecycle.compose.collectAsStateWithLifecycle
import org.osmdroid.config.Configuration
import uniffi.ride_ffi.SpeedScale
import uniffi.ride_ffi.accelBoundsMps2
import uniffi.ride_ffi.accelEdgeColorArgb
import uniffi.ride_ffi.speedEdgeColorArgb
import uniffi.ride_ffi.speedScaleBandsKmh
import uniffi.ride_ffi.speedScaleForTopSpeed
import uniffi.ride_ffi.speedScaleName
import org.osmdroid.events.MapEventsReceiver
import org.osmdroid.tileprovider.tilesource.TileSourceFactory
import org.osmdroid.util.GeoPoint
import org.osmdroid.views.MapView
import org.osmdroid.views.overlay.FolderOverlay
import org.osmdroid.views.overlay.MapEventsOverlay
import org.osmdroid.views.overlay.Polyline
import java.io.File
import java.text.DateFormat
import java.util.Date
import kotlin.math.hypot

/**
 * The live map: the current recording's GPS path colored by speed or acceleration, following the newest fix.
 * Dragging the map stops following; "Follow" resumes it. Tapping the path shows the speed and acceleration there.
 */
@Composable
fun LiveMapScreen(onBack: () -> Unit) {
    val points by LiveTrack.points.collectAsStateWithLifecycle()
    var follow by remember { mutableStateOf(true) }
    var selected by remember { mutableStateOf<LiveTrack.Point?>(null) }
    // Null is automatic, from the top speed so far.
    var chosenScale by rememberSaveable { mutableStateOf<SpeedScale?>(null) }
    var colorByAccel by rememberSaveable { mutableStateOf(false) }
    // Scales and colors come from Rust (`crates/fusion/src/speed_colors.rs`), the same as `ride map`.
    val scale = try {
        ScaleInfo(chosenScale ?: speedScaleForTopSpeed(points.mapNotNull { it.speedMps }.maxOrNull()))
    } catch (e: Throwable) {
        Log.e("RideLogger", "speed scales failed", e)
        Column(Modifier.padding(16.dp)) {
            Button(onClick = onBack) { Text("Back") }
            Text("Map colors failed to load (recording still works):\n${Log.getStackTraceString(e)}")
        }
        return
    }

    Column(Modifier.fillMaxSize().padding(16.dp), verticalArrangement = Arrangement.spacedBy(8.dp)) {
        Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
            Button(onClick = onBack) { Text("Back") }
            Button(onClick = { follow = true }, enabled = !follow) { Text("Follow") }
            Button(onClick = { colorByAccel = !colorByAccel }) { Text(if (colorByAccel) "Color: accel" else "Color: speed") }
            // Cycles automatic, then each scale.
            if (!colorByAccel) {
                Button(onClick = {
                    val choices = listOf(null) + SpeedScale.entries
                    chosenScale = choices[(choices.indexOf(chosenScale) + 1) % choices.size]
                }) { Text("Scale: " + if (chosenScale == null) "auto" else scale.name) }
            }
        }
        Text(describe(selected ?: points.lastOrNull(), latest = selected == null), style = MaterialTheme.typography.bodyMedium)
        Text(
            (
                if (colorByAccel) {
                    "Color: acceleration, orange braking through gray to purple speeding up, " +
                        "in bands at ${scale.accelBoundsMps2.joinToString(", ") { "%+.2f".format(Units.accel(it)) }} ${Units.accelUnit}; "
                } else {
                    "Color: speed, light to dark in bands at " +
                        scale.bandsKmh.joinToString(", ") { "%.0f".format(Units.speedFromKmh(it.toDouble())) } +
                        "+ ${Units.speedUnit} (${scale.name} scale); "
                }
            ) + "dark gray: no live speed; dashed: no GPS (tunnel)",
            style = MaterialTheme.typography.bodySmall,
        )
        // The map view draws past its own bounds (over the text above, and flickering below while tiles load).
        Box(Modifier.fillMaxWidth().weight(1f).clipToBounds()) {
            val track = remember { TrackOverlays() }
            AndroidView(
                modifier = Modifier.fillMaxSize(),
                factory = { context ->
                    // A map failure shows as text here; it can't affect recording.
                    try {
                        newMapView(context, track, onDrag = { follow = false }, onTap = { selected = it })
                            .apply { clipToOutline = true }
                    } catch (e: Throwable) {
                        Log.e("RideLogger", "map failed", e)
                        TextView(context).apply { text = "Map failed to load:\n${Log.getStackTraceString(e)}" }
                    }
                },
                update = { view ->
                    if (view !is MapView) return@AndroidView
                    try {
                        track.sync(points, scale.scale, colorByAccel)
                        val last = points.lastOrNull()
                        if (follow && last != null) view.controller.animateTo(GeoPoint(last.lat, last.lon))
                        view.invalidate()
                    } catch (e: Throwable) {
                        Log.e("RideLogger", "map update failed", e)
                    }
                },
                onRelease = { (it as? MapView)?.onDetach() },
            )
        }
    }
}

private fun describe(p: LiveTrack.Point?, latest: Boolean): String {
    if (p == null) return "No GPS fix yet this recording."
    fun f(v: Double?, digits: Int) = v?.let { "%.${digits}f".format(it) } ?: "unknown"
    val time = DateFormat.getTimeInstance().format(Date(p.timeMs))
    return (if (latest) "Now ($time): " else "At $time: ") +
        "${f(p.speedMps?.let(Units::speed), 1)} ${Units.speedUnit} · " +
        "${p.accelMps2?.let { "%+.2f".format(Units.accel(it)) } ?: "unknown"} ${Units.accelUnit}" +
        "\nsmoothed ${f(p.smoothedSpeedMps?.let(Units::speed), 1)} · live ${f(p.fusedSpeedMps?.let(Units::speed), 1)} · " +
        "GPS ${f(p.gpsSpeedMps?.let(Units::speed), 1)} ± ${f(p.gpsSpeedAccuracyMps?.let(Units::speed), 1)} ${Units.speedUnit}" +
        " · position ± ${p.accuracyM?.let(Units::distance) ?: "unknown"}"
}

@SuppressLint("ClickableViewAccessibility")
private fun newMapView(
    context: Context,
    track: TrackOverlays,
    onDrag: () -> Unit,
    onTap: (LiveTrack.Point?) -> Unit,
): MapView {
    Configuration.getInstance().apply {
        userAgentValue = context.packageName
        // The app's own cache, which needs no storage permission and is never mixed with recordings.
        osmdroidBasePath = File(context.cacheDir, "osmdroid")
        osmdroidTileCache = File(context.cacheDir, "osmdroid/tiles")
    }
    val map = MapView(context)
    map.setTileSource(TileSourceFactory.MAPNIK)
    map.setMultiTouchControls(true)
    map.controller.setZoom(16.0)
    map.setOnTouchListener { _, event ->
        if (event.actionMasked == MotionEvent.ACTION_MOVE) onDrag()
        false
    }
    map.overlays.add(
        MapEventsOverlay(object : MapEventsReceiver {
            override fun singleTapConfirmedHelper(p: GeoPoint): Boolean {
                onTap(track.nearest(map, p))
                return true
            }

            override fun longPressHelper(p: GeoPoint) = false
        }),
    )
    map.overlays.add(track.gaps)
    map.overlays.add(track.casings)
    map.overlays.add(track.colored)
    return map
}

/** The path's overlays, extended as fixes arrive instead of rebuilt. */
private class TrackOverlays {
    val gaps = FolderOverlay()
    val casings = FolderOverlay()
    val colored = FolderOverlay()
    private var drawn = emptyList<LiveTrack.Point>()
    private var scale: SpeedScale? = null
    private var byAccel = false
    private var casing: Polyline? = null
    private var line: Polyline? = null
    private var lineColor = 0

    fun sync(points: List<LiveTrack.Point>, scale: SpeedScale, byAccel: Boolean) {
        // A new recording, smoothing changed speeds already drawn, a faster top speed moved the ride to a wider
        // scale, or switching between speed and acceleration: start over. Otherwise only the new points are added.
        val restart = points.size < drawn.size || points.subList(0, drawn.size) != drawn
        if (restart || scale != this.scale || byAccel != this.byAccel) {
            this.scale = scale
            this.byAccel = byAccel
            listOf(gaps, casings, colored).forEach { it.items.clear() }
            drawn = emptyList()
            casing = null
            line = null
        }
        for (i in drawn.size until points.size) add(points.getOrNull(i - 1), points[i])
        drawn = points
    }

    private fun add(prev: LiveTrack.Point?, p: LiveTrack.Point) {
        val geo = GeoPoint(p.lat, p.lon)
        if (prev == null || p.startsSegment) {
            if (prev != null) {
                gaps.add(polyline(Color.GRAY, 2f).apply {
                    outlinePaint.pathEffect = DashPathEffect(floatArrayOf(12f, 12f), 0f)
                    setPoints(listOf(GeoPoint(prev.lat, prev.lon), geo))
                })
            }
            casing = polyline(Color.WHITE, 16f).also { it.addPoint(geo); casings.add(it) }
            line = null
            return
        }
        casing?.addPoint(geo)
        val color = if (byAccel) {
            accelEdgeColorArgb(prev.accelMps2, p.accelMps2).toInt()
        } else {
            speedEdgeColorArgb(scale!!, prev.speedMps, p.speedMps).toInt()
        }
        val current = line
        if (current != null && color == lineColor) {
            current.addPoint(geo)
        } else {
            line = polyline(color, 9f).also {
                it.setPoints(listOf(GeoPoint(prev.lat, prev.lon), geo))
                colored.add(it)
            }
            lineColor = color
        }
    }

    private fun polyline(color: Int, width: Float) = Polyline().apply {
        outlinePaint.color = color
        outlinePaint.strokeWidth = width
        outlinePaint.strokeCap = android.graphics.Paint.Cap.ROUND
        isEnabled = true
        setOnClickListener { _, _, _ -> false }
    }

    /** The fix nearest a tap, if it's within a finger's width. */
    fun nearest(map: MapView, tap: GeoPoint): LiveTrack.Point? {
        val projection = map.projection
        val t = projection.toPixels(tap, null)
        val maxPx = 48 * map.resources.displayMetrics.density
        return drawn.minByOrNull {
            val q = projection.toPixels(GeoPoint(it.lat, it.lon), null)
            hypot((q.x - t.x).toDouble(), (q.y - t.y).toDouble())
        }?.takeIf {
            val q = projection.toPixels(GeoPoint(it.lat, it.lon), null)
            hypot((q.x - t.x).toDouble(), (q.y - t.y).toDouble()) <= maxPx
        }
    }
}

/** A scale with what the screen shows about it, from Rust. */
private class ScaleInfo(val scale: SpeedScale) {
    val name = speedScaleName(scale)
    val bandsKmh = speedScaleBandsKmh(scale)
    val accelBoundsMps2 = accelBoundsMps2()
}
