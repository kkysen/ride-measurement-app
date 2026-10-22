package dev.khyber.ride

import android.content.Context
import android.net.ConnectivityManager
import android.net.Network
import android.net.NetworkCapabilities
import android.util.Log
import dev.khyber.ride.proto.GtfsRealtimeFeed
import java.io.IOException
import java.io.InterruptedIOException
import java.net.ConnectException
import java.net.HttpURLConnection
import java.net.NoRouteToHostException
import java.net.SocketTimeoutException
import java.net.URL
import java.net.UnknownHostException
import uniffi.ride_ffi.gtfsRtFeeds
import uniffi.ride_ffi.gtfsRtPollMs

/**
 * Records the MTA's subway GTFS-realtime feeds while recording, for which trip and train the rider was on.
 * Rust picks the feeds (`ride_fusion::gtfs_rt`: the lines near [LiveSpeed.lastPosition]) and how often;
 * this only does the HTTP requests and stores each response unchanged in the `transit` stream.
 * Having no network, as underground, is only logged; recording never depends on it.
 * Underground the network comes and goes in stations, often for seconds: a poll that failed is retried
 * the moment Android says the network is back (validated), not on the next 30 s tick,
 * with short timeouts and the nearest lines' feeds first so a brief window gets the rider's own line.
 */
class GtfsRtFetcher(context: Context, private val records: RecordStreamRecorder) {
    @Volatile
    private var running = true
    private val wake = Object()
    private var woken = false
    private val connectivity = context.getSystemService(ConnectivityManager::class.java)
    private val callback = object : ConnectivityManager.NetworkCallback() {
        override fun onCapabilitiesChanged(network: Network, capabilities: NetworkCapabilities) {
            if (capabilities.hasCapability(NetworkCapabilities.NET_CAPABILITY_VALIDATED)) wakeUp()
        }
    }
    private val thread = Thread({ loop() }, "gtfs-rt-fetcher").apply {
        isDaemon = true
        start()
    }

    init {
        Health.guard("gtfs-rt network callback") { connectivity?.registerDefaultNetworkCallback(callback) }
    }

    fun stop() {
        running = false
        Health.guard("gtfs-rt network callback") { connectivity?.unregisterNetworkCallback(callback) }
        thread.interrupt()
    }

    private fun wakeUp() = synchronized(wake) {
        woken = true
        wake.notifyAll()
    }

    private fun loop() {
        while (running) {
            val position = LiveSpeed.lastPosition
            val urls = Health.guard("gtfs-rt plan") { gtfsRtFeeds(position?.first, position?.second) }
            var allPolled = true
            for (url in urls.orEmpty()) {
                if (!running) return
                val ok = Health.guard("gtfs-rt download") {
                    try {
                        poll(url)
                        true
                    } catch (e: InterruptedIOException) {
                        // Stopping the recording interrupts a request in flight; a timeout means the window closed.
                        if (!running) return
                        if (e !is SocketTimeoutException) throw e
                        Log.i("RideLogger", "gtfs-rt: timed out: $url")
                        false
                    } catch (e: IOException) {
                        if (e !is UnknownHostException && e !is ConnectException && e !is NoRouteToHostException) throw e
                        Log.i("RideLogger", "gtfs-rt: offline: $e")
                        false
                    }
                } == true
                // Offline: the rest would fail too; wait for the network instead.
                if (!ok) {
                    allPolled = false
                    break
                }
            }
            val pollMs = Health.guard("gtfs-rt poll") { gtfsRtPollMs() } ?: FALLBACK_POLL_MS
            try {
                synchronized(wake) {
                    // After a failure, the network coming back ends the wait early; after success, only the timer.
                    if (allPolled) woken = false
                    if (!woken) wake.wait(pollMs)
                    woken = false
                }
            } catch (_: InterruptedException) {
                return
            }
        }
    }

    private fun poll(url: String) {
        val requestMs = System.currentTimeMillis()
        val connection = URL(url).openConnection() as HttpURLConnection
        connection.connectTimeout = TIMEOUT_MS
        connection.readTimeout = TIMEOUT_MS
        try {
            val status = connection.responseCode
            val body = (if (status < 400) connection.inputStream else connection.errorStream)?.use { it.readBytes() }
            val feed = GtfsRealtimeFeed.newBuilder()
                .setUrl(url)
                .setRequestUnixMs(requestMs)
                .setHttpStatus(status)
                .setBody(com.google.protobuf.ByteString.copyFrom(body ?: ByteArray(0)))
                .build()
            records.add { gtfsRealtime = feed }
        } finally {
            connection.disconnect()
        }
    }

    private companion object {
        /** Short: a station's signal can last only seconds, and a stalled request would hold up the next feed. */
        const val TIMEOUT_MS = 5_000
        /** Only if Rust can't say. */
        const val FALLBACK_POLL_MS = 30_000L
    }
}
