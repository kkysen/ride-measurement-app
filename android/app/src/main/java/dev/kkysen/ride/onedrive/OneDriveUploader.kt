package dev.kkysen.ride.onedrive

import java.io.File
import java.io.RandomAccessFile
import java.net.URLEncoder
import org.json.JSONObject

/**
 * Uploads files into the app's OneDrive folder (`Apps/<app name>/`) with resumable upload sessions,
 * and checks each upload's `quickXorHash` against one computed locally.
 */
class OneDriveUploader(private val auth: OneDriveAuth) {
    /** Uploads [file] to `<remoteDir>/<file name>` and returns once its remote hash is verified. */
    fun uploadVerified(file: File, remoteDir: String) {
        val token = auth.accessToken()
        val localHash = hashOf(file)
        val path = "$remoteDir/${file.name}".split("/").joinToString("/") { URLEncoder.encode(it, "UTF-8").replace("+", "%20") }
        val session = Http.request(
            "POST",
            "$GRAPH/me/drive/special/approot:/$path:/createUploadSession",
            JSONObject().put("item", JSONObject().put("@microsoft.graph.conflictBehavior", "replace")).toString().toByteArray(),
            mapOf("Authorization" to "Bearer $token", "Content-Type" to "application/json"),
        )
        val uploadUrl = session.getString("uploadUrl")
        val size = file.length()
        var item = JSONObject()
        RandomAccessFile(file, "r").use { raf ->
            val buf = ByteArray(FRAGMENT_BYTES)
            var offset = 0L
            // An empty file still needs one (empty) request; Graph rejects zero-length fragments, so use simple upload.
            if (size == 0L) {
                item = Http.request(
                    "PUT",
                    "$GRAPH/me/drive/special/approot:/$path:/content",
                    ByteArray(0),
                    mapOf("Authorization" to "Bearer $token"),
                )
            }
            while (offset < size) {
                val n = minOf(FRAGMENT_BYTES.toLong(), size - offset).toInt()
                raf.seek(offset)
                raf.readFully(buf, 0, n)
                // The upload URL is pre-authorized; Graph rejects an Authorization header on it.
                item = Http.request(
                    "PUT",
                    uploadUrl,
                    buf.copyOf(n),
                    mapOf("Content-Range" to "bytes $offset-${offset + n - 1}/$size"),
                )
                offset += n
            }
        }
        val remoteHash = item.optJSONObject("file")?.optJSONObject("hashes")?.optString("quickXorHash")
        check(remoteHash == localHash) { "hash mismatch for ${file.name}: local $localHash, remote $remoteHash" }
    }

    private fun hashOf(file: File): String {
        val hash = QuickXorHash()
        file.inputStream().use { input ->
            val buf = ByteArray(1 shl 16)
            while (true) {
                val n = input.read(buf)
                if (n < 0) break
                hash.update(buf, 0, n)
            }
        }
        return hash.digestBase64()
    }

    companion object {
        private const val GRAPH = "https://graph.microsoft.com/v1.0"

        /** Graph requires fragments to be multiples of 320 KiB. */
        private const val FRAGMENT_BYTES = 320 * 1024 * 16
    }
}
