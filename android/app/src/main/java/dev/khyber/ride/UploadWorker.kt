package dev.khyber.ride

import android.content.Context
import androidx.work.Constraints
import androidx.work.CoroutineWorker
import androidx.work.ExistingWorkPolicy
import androidx.work.NetworkType
import androidx.work.OneTimeWorkRequestBuilder
import androidx.work.WorkManager
import androidx.work.WorkerParameters
import dev.khyber.ride.onedrive.OneDriveAuth
import dev.khyber.ride.onedrive.OneDriveUploader
import java.io.File
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.withContext

/**
 * Uploads every finished session not yet uploaded. Nothing is ever deleted here:
 * a verified upload only writes an `uploaded` marker, and deletion comes later, after a second verification.
 */
class UploadWorker(context: Context, params: WorkerParameters) : CoroutineWorker(context, params) {
    override suspend fun doWork(): Result = withContext(Dispatchers.IO) {
        val auth = OneDriveAuth(applicationContext)
        if (!auth.configured || !auth.signedIn) return@withContext Result.success()
        val uploader = OneDriveUploader(auth)
        val current = RecorderService.state.value.sessionId
        val sessions = RecorderService.sessionsDir().listFiles().orEmpty()
            .filter { it.isDirectory && it.name != current && !File(it, UPLOADED_MARKER).exists() }
        try {
            for (session in sessions.sortedBy { it.name }) {
                for (file in session.listFiles().orEmpty().filter { it.isFile }.sortedBy { it.name }) {
                    uploader.uploadVerified(file, "sessions/${session.name}")
                }
                File(session, UPLOADED_MARKER).writeText("${System.currentTimeMillis()}\n")
            }
            Result.success()
        } catch (e: Exception) {
            android.util.Log.w("UploadWorker", "upload failed, will retry", e)
            Result.retry()
        }
    }

    companion object {
        const val UPLOADED_MARKER = ".uploaded"

        fun enqueue(context: Context) {
            val request = OneTimeWorkRequestBuilder<UploadWorker>()
                .setConstraints(Constraints.Builder().setRequiredNetworkType(NetworkType.UNMETERED).build())
                .build()
            WorkManager.getInstance(context).enqueueUniqueWork("upload", ExistingWorkPolicy.APPEND_OR_REPLACE, request)
        }
    }
}
