package dev.khyber.ride

import java.util.TreeMap
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import uniffi.ride_ffi.LiveStop
import uniffi.ride_ffi.SmoothedSpeed
import uniffi.ride_ffi.StopsDelta
import uniffi.ride_ffi.TimelineDelta
import uniffi.ride_ffi.TimelineSegment

/**
 * The current recording's speed and acceleration over time, for the live graph, underground too
 * (the map only has GPS fixes): the smoothed values where smoothing has reached, else the live estimate.
 * Kept after a recording stops, like [LiveTrack], until the next one starts.
 */
object LiveSeries {
    data class Sample(val elapsedRealtimeNs: Long, val speedMps: Double, val accelMps2: Double)

    /** At most one live sample this often; smoothed ones come about once a second anyway. */
    private const val LIVE_EVERY_NS = 1_000_000_000L

    private val smoothed = TreeMap<Long, Sample>()
    private val live = TreeMap<Long, Sample>()

    private val _samples = MutableStateFlow<List<Sample>>(emptyList())
    val samples: StateFlow<List<Sample>> = _samples
    private val stopsCopy = SettledList<LiveStop>()
    private val timelineCopy = SettledList<TimelineSegment>()
    private val _stops = MutableStateFlow<List<LiveStop>>(emptyList())
    val stops: StateFlow<List<LiveStop>> = _stops
    private val _timeline = MutableStateFlow<List<TimelineSegment>>(emptyList())
    val timeline: StateFlow<List<TimelineSegment>> = _timeline

    @Synchronized
    fun reset() {
        smoothed.clear()
        live.clear()
        stopsCopy.clear()
        timelineCopy.clear()
        _samples.value = emptyList()
        _stops.value = emptyList()
        _timeline.value = emptyList()
    }

    @Synchronized
    fun addLive(elapsedRealtimeNs: Long, speedMps: Double, accelMps2: Double) {
        val last = live.lastEntry()
        if (last != null && elapsedRealtimeNs - last.key < LIVE_EVERY_NS) return
        live[elapsedRealtimeNs] = Sample(elapsedRealtimeNs, speedMps, accelMps2)
        publish()
    }

    @Synchronized
    fun applySmoothed(values: List<SmoothedSpeed>) {
        if (values.isEmpty()) return
        for (v in values) smoothed[v.elapsedRealtimeNs] = Sample(v.elapsedRealtimeNs, v.speedMps, v.accelMps2)
        publish()
    }

    /** How many stops are settled in the copy here: what to ask `Fusion.stopsSince` from. */
    val stopsSettled: UInt
        @Synchronized get() = stopsCopy.settled

    /** How many timeline segments are settled in the copy here: what to ask `Fusion.timelineSince` from. */
    val timelineSettled: UInt
        @Synchronized get() = timelineCopy.settled

    @Synchronized
    fun applyStops(d: StopsDelta) {
        if (stopsCopy.apply(d.from, d.settled, d.items)) _stops.value = stopsCopy.snapshot()
    }

    @Synchronized
    fun applyTimeline(d: TimelineDelta) {
        if (timelineCopy.apply(d.from, d.settled, d.items)) _timeline.value = timelineCopy.snapshot()
    }

    /** Smoothed values up to the latest one, then live ones after it. */
    private fun publish() {
        val end = smoothed.lastEntry()?.key
        val tail = if (end == null) live.values else live.tailMap(end, false).values
        _samples.value = smoothed.values + tail
    }
}
