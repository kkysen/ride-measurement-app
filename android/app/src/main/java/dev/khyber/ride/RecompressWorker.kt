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

/** Recompresses finished sessions (see [Recompressor]), then hands off to [UploadWorker]. */
class RecompressWorker(context: Context, params: WorkerParameters) : CoroutineWorker(context, params) {
    override suspend fun doWork(): Result = withContext(Dispatchers.IO) {
        val current = RecorderService.state.value.sessionId
        RecorderService.sessionsDir().listFiles().orEmpty()
            .filter { it.isDirectory && it.name != current && !File(it, Recompressor.MARKER).exists() }
            .sortedBy { it.name }
            .forEach { session ->
                for ((name, result) in Recompressor.recompressSession(session)) Log.i("RecompressWorker", "${session.name}/$name: $result")
            }
        Result.success()
    }

    companion object {
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
