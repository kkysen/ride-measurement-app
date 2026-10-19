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
import androidx.compose.runtime.setValue
import androidx.compose.ui.Modifier
import androidx.compose.ui.unit.dp
import androidx.compose.ui.viewinterop.AndroidView
import androidx.lifecycle.compose.collectAsStateWithLifecycle
import org.osmdroid.config.Configuration
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
 * The live map: the current recording's GPS path colored by speed, following the newest fix.
 * Dragging the map stops following; "Follow" resumes it. Tapping the path shows the speed there.
 */
@Composable
fun LiveMapScreen(onBack: () -> Unit) {
    val points by LiveTrack.points.collectAsStateWithLifecycle()
    var follow by remember { mutableStateOf(true) }
    var selected by remember { mutableStateOf<LiveTrack.Point?>(null) }

    Column(Modifier.fillMaxSize().padding(16.dp), verticalArrangement = Arrangement.spacedBy(8.dp)) {
        Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
            Button(onClick = onBack) { Text("Back") }
            Button(onClick = { follow = true }, enabled = !follow) { Text("Follow") }
        }
        Text(describe(selected ?: points.lastOrNull(), latest = selected == null), style = MaterialTheme.typography.bodyMedium)
        Text(
            "Color: speed, light 0 to dark ${SpeedColors.MAX_KMH.toInt()}+ km/h; dashed: no GPS (tunnel)",
            style = MaterialTheme.typography.bodySmall,
        )
        Box(Modifier.fillMaxWidth().weight(1f)) {
            val track = remember { TrackOverlays() }
            AndroidView(
                modifier = Modifier.fillMaxSize(),
                factory = { context ->
                    // A map failure shows as text here; it can't affect recording.
                    try {
                        newMapView(context, track, onDrag = { follow = false }, onTap = { selected = it })
                    } catch (e: Throwable) {
                        Log.e("RideLogger", "map failed", e)
                        TextView(context).apply { text = "Map failed to load:\n${Log.getStackTraceString(e)}" }
                    }
                },
                update = { view ->
                    if (view !is MapView) return@AndroidView
                    try {
                        track.sync(points)
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
        "${f(p.speedMps?.times(3.6), 1)} km/h" +
        "\nfused ${f(p.fusedSpeedMps, 2)} m/s · GPS ${f(p.gpsSpeedMps, 2)} ± ${f(p.gpsSpeedAccuracyMps, 2)} m/s" +
        " · position ± ${f(p.accuracyM, 0)} m"
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
    private var casing: Polyline? = null
    private var line: Polyline? = null
    private var lineColor = 0

    fun sync(points: List<LiveTrack.Point>) {
        // A new recording: start over.
        if (points.size < drawn.size || (drawn.isNotEmpty() && points.firstOrNull() !== drawn.first())) {
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
                gaps.add(polyline(Color.rgb(0x8A, 0x89, 0x84), 2f).apply {
                    outlinePaint.pathEffect = DashPathEffect(floatArrayOf(12f, 12f), 0f)
                    setPoints(listOf(GeoPoint(prev.lat, prev.lon), geo))
                })
            }
            casing = polyline(Color.WHITE, 16f).also { it.addPoint(geo); casings.add(it) }
            line = null
            return
        }
        casing?.addPoint(geo)
        val speeds = listOfNotNull(prev.speedMps, p.speedMps)
        val color = SpeedColors.of(if (speeds.size == 2) speeds.average() else null)
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
