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
import androidx.compose.foundation.layout.safeDrawingPadding
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
import java.io.File
import java.text.DateFormat
import java.util.Date
import kotlin.math.hypot
import org.osmdroid.config.Configuration
import org.osmdroid.events.MapEventsReceiver
import org.osmdroid.tileprovider.tilesource.TileSourceFactory
import org.osmdroid.util.BoundingBox
import org.osmdroid.util.GeoPoint
import org.osmdroid.views.MapView
import org.osmdroid.views.overlay.FolderOverlay
import org.osmdroid.views.overlay.MapEventsOverlay
import org.osmdroid.views.overlay.Polyline
import uniffi.ride_ffi.SpeedScale
import uniffi.ride_ffi.accelBoundsMps2
import uniffi.ride_ffi.accelEdgeColorArgb
import uniffi.ride_ffi.speedEdgeColorArgb
import uniffi.ride_ffi.speedScaleBandsKmh
import uniffi.ride_ffi.speedScaleForTopSpeed
import uniffi.ride_ffi.speedScaleName

/**
 * The live map: the current recording's GPS path colored by speed or acceleration, following the newest fix.
 * Dragging the map stops following; "Follow" resumes it. Tapping the path shows the speed and acceleration there.
 */
@Composable
fun LiveMapScreen(onBack: () -> Unit) {
    val points by LiveTrack.points.collectAsStateWithLifecycle()
    TrackMapScreen(points, live = true, onBack)
}

/**
 * A GPS path colored by speed or acceleration: the current recording's ([live], following the newest fix),
 * or a past ride's, shown whole.
 * A past ride's [points] are its fused path (joined to the subway track underground), and [gpsFixes] the GPS fixes
 * themselves, drawn as a thin dark line over it.
 */
@Composable
fun TrackMapScreen(
    points: List<LiveTrack.Point>,
    live: Boolean,
    onBack: () -> Unit,
    extraButton: @Composable () -> Unit = {},
    gpsFixes: List<LiveTrack.Point> = emptyList(),
) {
    var follow by remember { mutableStateOf(live) }
    var selected by remember { mutableStateOf<LiveTrack.Point?>(null) }
    // Null is automatic, from the top speed so far.
    var chosenScale by rememberSaveable { mutableStateOf<SpeedScale?>(null) }
    var colorByAccel by rememberSaveable { mutableStateOf(false) }
    // Scales and colors come from Rust (`crates/fusion/src/speed_colors.rs`), the same as `ride map`.
    val scale = try {
        ScaleInfo(chosenScale ?: speedScaleForTopSpeed(points.mapNotNull { it.speedMps }.maxOrNull()))
    } catch (e: Throwable) {
        Log.e("RideLogger", "speed scales failed", e)
        Column(Modifier.safeDrawingPadding().padding(16.dp)) {
            HapticButton(onClick = onBack) { Text("Back") }
            Text("Map colors failed to load (recording still works):\n${Log.getStackTraceString(e)}")
        }
        return
    }

    Column(Modifier.fillMaxSize().safeDrawingPadding().padding(16.dp), verticalArrangement = Arrangement.spacedBy(8.dp)) {
        Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
            HapticButton(onClick = onBack) { Text("Back") }
            extraButton()
            if (live) HapticButton(onClick = { follow = true }, enabled = !follow) { Text("Follow") }
            HapticButton(onClick = { colorByAccel = !colorByAccel }) { Text(if (colorByAccel) "Color: accel" else "Color: speed") }
            // Cycles automatic, then each scale.
            if (!colorByAccel) {
                HapticButton(onClick = {
                    val choices = listOf(null) + SpeedScale.entries
                    chosenScale = choices[(choices.indexOf(chosenScale) + 1) % choices.size]
                }) { Text("Scale: " + if (chosenScale == null) "auto" else scale.name) }
            }
        }
        Text(describe(selected ?: points.lastOrNull(), latest = selected == null, live), style = MaterialTheme.typography.bodyMedium)
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
            ) + "dark gray: no live speed; dashed: no GPS (tunnel)" +
                if (gpsFixes.isNotEmpty()) "; thin dark line: GPS fixes" else "",
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
                            .apply {
                                clipToOutline = true
                                // A past ride doesn't move: show all of it.
                                if (!live && points.isNotEmpty()) {
                                    val box = BoundingBox.fromGeoPoints(points.map { GeoPoint(it.lat, it.lon) })
                                    addOnFirstLayoutListener { _, _, _, _, _ -> zoomToBoundingBox(box, false, 48) }
                                }
                            }
                    } catch (e: Throwable) {
                        Log.e("RideLogger", "map failed", e)
                        TextView(context).apply { text = "Map failed to load:\n${Log.getStackTraceString(e)}" }
                    }
                },
                update = { view ->
                    if (view !is MapView) return@AndroidView
                    try {
                        track.sync(points, scale.scale, colorByAccel)
                        track.syncGps(gpsFixes)
                        val last = points.lastOrNull()
                        // Pans only once the newest fix nears the edge, so the map isn't always moving and
                        // cancelling tiles that are still loading.
                        if (follow && last != null && !nearCenter(view, last)) view.controller.animateTo(GeoPoint(last.lat, last.lon))
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

/** Whether a point is in the middle 60% of the visible map, both ways. */
private fun nearCenter(view: MapView, p: LiveTrack.Point): Boolean {
    val box = view.boundingBox
    if (box.latitudeSpan <= 0.0 || box.longitudeSpanWithDateLine <= 0.0) return false
    val y = (box.latNorth - p.lat) / box.latitudeSpan
    val x = (p.lon - box.lonWest) / box.longitudeSpanWithDateLine
    return x in 0.2..0.8 && y in 0.2..0.8
}

private fun describe(p: LiveTrack.Point?, latest: Boolean, live: Boolean): String {
    if (p == null) return if (live) "No GPS fix yet this recording." else "No GPS fixes on this ride."
    fun f(v: Double?, digits: Int) = v?.let { "%.${digits}f".format(it) } ?: "unknown"
    val time = DateFormat.getTimeInstance().format(Date(p.timeMs))
    return (if (latest) "${if (live) "Now" else "Last fix"} ($time): " else "At $time: ") +
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
        // The app's own files, which need no storage permission and are never mixed with recordings.
        // Not the cache directory: Android clears that when space runs low, and every tile then comes over the network
        // again, which is slow underground or with throttled Wi-Fi.
        osmdroidBasePath = File(context.filesDir, "osmdroid")
        osmdroidTileCache = File(context.filesDir, "osmdroid/tiles")
        tileFileSystemCacheMaxBytes = 1L shl 30
        tileFileSystemCacheTrimBytes = 900L shl 20
        // Streets rarely change: show a cached tile for a month before asking the server for a newer one.
        expirationOverrideDuration = 30L * 24 * 60 * 60 * 1000
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
    map.overlays.add(track.gps)
    return map
}

/** The path's overlays, extended as fixes arrive instead of rebuilt. */
private class TrackOverlays {
    val gaps = FolderOverlay()
    val casings = FolderOverlay()
    val colored = FolderOverlay()
    /** The GPS fixes alone, one thin line per segment, when the path is the fused one. */
    val gps = FolderOverlay()
    private var gpsDrawn = emptyList<LiveTrack.Point>()
    private var drawn = emptyList<LiveTrack.Point>()
    /** After each drawn point: how many overlays there were, and how many points the open lines had. */
    private data class Mark(val gaps: Int, val casings: Int, val colored: Int, val casingPoints: Int, val linePoints: Int, val lineColor: Int)
    private val marks = ArrayList<Mark>()
    private var scale: SpeedScale? = null
    private var byAccel = false
    private var casing: Polyline? = null
    private var line: Polyline? = null
    private var lineColor = 0

    fun sync(points: List<LiveTrack.Point>, scale: SpeedScale, byAccel: Boolean) {
        // A faster top speed moved the ride to a wider scale, or switching between speed and acceleration:
        // start over. Otherwise the path is redrawn from the first point that changed (smoothing changes only the
        // last few minutes' speeds; a new recording changes the first), and new points are added.
        if (scale != this.scale || byAccel != this.byAccel) {
            this.scale = scale
            this.byAccel = byAccel
            truncate(0)
        }
        var same = 0
        val n = minOf(points.size, drawn.size)
        while (same < n && points[same] == drawn[same]) same++
        if (same < drawn.size) truncate(same)
        for (i in drawn.size until points.size) {
            add(points.getOrNull(i - 1), points[i])
            marks.add(Mark(gaps.items.size, casings.items.size, colored.items.size, casing?.actualPoints?.size ?: 0, line?.actualPoints?.size ?: 0, lineColor))
        }
        drawn = points
    }

    fun syncGps(fixes: List<LiveTrack.Point>) {
        if (fixes == gpsDrawn) return
        gps.items.clear()
        var line: Polyline? = null
        for (p in fixes) {
            if (line == null || p.startsSegment) line = polyline(Color.DKGRAY, 3f).also { gps.add(it) }
            line.addPoint(GeoPoint(p.lat, p.lon))
        }
        gpsDrawn = fixes
    }

    /** Back to just after the first `n` points were drawn. */
    private fun truncate(n: Int) {
        val m = marks.getOrNull(n - 1)
        if (m == null) {
            listOf(gaps, casings, colored).forEach { it.items.clear() }
            marks.clear()
            drawn = emptyList()
            casing = null
            line = null
            return
        }
        fun FolderOverlay.keep(k: Int) {
            while (items.size > k) items.removeAt(items.size - 1)
        }
        gaps.keep(m.gaps)
        casings.keep(m.casings)
        colored.keep(m.colored)
        casing = (casings.items.lastOrNull() as Polyline?)?.also { it.setPoints(it.actualPoints.take(m.casingPoints)) }
        line = if (m.linePoints == 0) null else (colored.items.lastOrNull() as Polyline?)?.also { it.setPoints(it.actualPoints.take(m.linePoints)) }
        lineColor = m.lineColor
        while (marks.size > n) marks.removeAt(marks.size - 1)
        drawn = drawn.subList(0, n)
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
