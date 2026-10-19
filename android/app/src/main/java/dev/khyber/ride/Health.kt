package dev.khyber.ride

import android.util.Log
import dev.khyber.ride.proto.RecorderError
import java.util.concurrent.ConcurrentHashMap
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow

/**
 * Errors while recording. Any one part failing (a sensor, an Android API, a write) is reported here and
 * everything else keeps going: [guard] wraps each callback and step, so an exception never reaches a recorder
 * thread and crashes the app, which would stop every stream.
 *
 * Each error is logged, shown on screen, and written into the session's `events` stream,
 * so analysis knows what's missing and why. Repeats are recorded at 1, 2, 4, 8, ... occurrences.
 */
object Health {
    data class Problem(val message: String, val count: Long)

    private val counts = ConcurrentHashMap<String, Long>()
    private val _problems = MutableStateFlow<Map<String, Problem>>(emptyMap())
    val problems: StateFlow<Map<String, Problem>> = _problems

    /** Where errors are recorded; set while a session is recording. */
    @Volatile
    var events: RecordStreamRecorder? = null

    fun reset() {
        counts.clear()
        _problems.value = emptyMap()
    }

    /** Runs [block]; if it throws, reports it under [source] and returns null. */
    inline fun <T> guard(source: String, block: () -> T): T? = try {
        block()
    } catch (e: Throwable) {
        report(source, e)
        null
    }

    fun report(source: String, e: Throwable) {
        val count = counts.merge(source, 1L, Long::plus) ?: 1L
        val message = "${e.javaClass.simpleName}: ${e.message}"
        synchronized(this) { _problems.value = _problems.value + (source to Problem(message, count)) }
        // Powers of two, so an error on every sensor sample doesn't flood the log or the session.
        if (count and (count - 1) != 0L) return
        Log.e("RideLogger", "$source failed ($count times)", e)
        try {
            events?.add {
                recorderError = RecorderError.newBuilder()
                    .setSource(source)
                    .setMessage(Log.getStackTraceString(e))
                    .setCount(count)
                    .build()
            }
        } catch (_: Throwable) {
            // The events stream is what's failing; the log and the screen still have it.
        }
    }
}
