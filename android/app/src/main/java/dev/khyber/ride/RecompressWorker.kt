package dev.khyber.ride

import android.content.Context
import android.util.Log
import androidx.work.Constraints
import androidx.work.CoroutineWorker
import androidx.work.ExistingWorkPolicy
import androidx.work.OneTimeWorkRequestBuilder
import androidx.work.WorkManager
import androidx.work.WorkerParameters
import java.io.File
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.withContext
import uniffi.ride_ffi.recompressSession
import uniffi.ride_ffi.recompressedMarker

/**
 * Recompresses finished sessions with the Rust recompressor (`crates/session/src/recompress.rs`),
 * then hands off to [UploadWorker].
 * Any failure keeps the original files; a session left unmarked is retried next time and not uploaded until then.
 */
class RecompressWorker(context: Context, params: WorkerParameters) : CoroutineWorker(context, params) {
    override suspend fun doWork(): Result = withContext(Dispatchers.IO) {
        val current = RecorderService.state.value.sessionId
        try {
            val marker = recompressedMarker()
            RecorderService.sessionsDir().listFiles().orEmpty()
                .filter { it.isDirectory && it.name != current && !File(it, marker).exists() }
                .sortedBy { it.name }
                .forEach { session ->
                    try {
                        for (r in recompressSession(session.path)) Log.i(TAG, "${session.name}/${r.fileName}: ${r.summary}")
                    } catch (e: Throwable) {
                        Log.e(TAG, "recompressing ${session.name} failed; originals kept", e)
                    }
                }
        } catch (e: Throwable) {
            // E.g. the Rust library failed to load: nothing was touched.
            Log.e(TAG, "recompression unavailable; originals kept", e)
        }
        Result.success()
    }

    companion object {
        private const val TAG = "RecompressWorker"

        /** Recompression, then upload, as one chain, so a session is never uploaded and then replaced. */
        fun enqueueThenUpload(context: Context) {
            val recompress = OneTimeWorkRequestBuilder<RecompressWorker>()
                .setConstraints(Constraints.Builder().setRequiresBatteryNotLow(true).build())
                .build()
            WorkManager.getInstance(context)
                .beginUniqueWork("recompress", ExistingWorkPolicy.APPEND_OR_REPLACE, recompress)
                .then(UploadWorker.request())
                .enqueue()
        }
    }
}
