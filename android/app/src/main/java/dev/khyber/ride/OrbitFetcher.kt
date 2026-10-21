package dev.khyber.ride

import android.util.Log
import java.io.File
import java.io.IOException
import java.net.ConnectException
import java.net.HttpURLConnection
import java.net.NoRouteToHostException
import java.net.URL
import java.net.UnknownHostException
import uniffi.ride_ffi.orbitFetchPlan
import uniffi.ride_ffi.orbitRefreshMs

/**
 * Downloads broadcast orbits while recording, for live speed from every constellation's satellites.
 * Rust decides everything (`ride_gnss::orbits::fetch_plan`, the same plan `just orbits` follows offline):
 * which files, from where, into the session's `orbits/`, how often; this only does the HTTP requests.
 * Each failure (a server error, a bad file) is reported and skipped; recording never depends on it.
 * Having no network at all, as underground, is only logged.
 */
class OrbitFetcher(private val sessionDir: File) {
    @Volatile
    private var running = true
    private val thread = Thread({ loop() }, "orbit-fetcher").apply {
        isDaemon = true
        start()
    }

    fun stop() {
        running = false
        thread.interrupt()
    }

    private fun loop() {
        // Files from before a restart of this session, if any.
        LiveSpeed.addSessionOrbits(sessionDir.path)
        while (running) {
            val plan = Health.guard("orbit plan") { orbitFetchPlan(sessionDir.path, System.currentTimeMillis()) }
            for (day in plan.orEmpty()) {
                for (file in day) {
                    if (!running) return
                    val downloaded = Health.guard("orbit download") {
                        try {
                            download(file.url, File(file.path))
                        } catch (e: IOException) {
                            // No network, as underground: normal, not a problem with recording; the next refresh retries.
                            if (e !is UnknownHostException && e !is ConnectException && e !is NoRouteToHostException) throw e
                            Log.i("RideLogger", "orbit download: offline: $e")
                            false
                        }
                    }
                    if (downloaded == true) {
                        LiveSpeed.addOrbitFile(file.path)
                        break
                    }
                }
            }
            try {
                Thread.sleep(Health.guard("orbit refresh") { orbitRefreshMs() } ?: FALLBACK_REFRESH_MS)
            } catch (_: InterruptedException) {
                return
            }
        }
    }

    /** Downloads [url] to [to] via a temporary file; false if the server doesn't have it. */
    private fun download(url: String, to: File): Boolean {
        val connection = URL(url).openConnection() as HttpURLConnection
        connection.connectTimeout = TIMEOUT_MS
        connection.readTimeout = TIMEOUT_MS
        try {
            if (connection.responseCode != HttpURLConnection.HTTP_OK) return false
            to.parentFile?.mkdirs()
            val part = File(to.path + ".part")
            connection.inputStream.use { input -> part.outputStream().use { input.copyTo(it) } }
            if (!part.renameTo(to)) error("couldn't rename $part to $to")
            return true
        } finally {
            connection.disconnect()
        }
    }

    private companion object {
        const val TIMEOUT_MS = 30_000
        /** Only if Rust can't say: BKG's update interval. */
        const val FALLBACK_REFRESH_MS = 15 * 60_000L
    }
}
