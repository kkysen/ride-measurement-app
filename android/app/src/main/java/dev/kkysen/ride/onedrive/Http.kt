package dev.kkysen.ride.onedrive

import java.net.HttpURLConnection
import java.net.URL
import java.net.URLEncoder
import org.json.JSONObject

internal object Http {
    fun postForm(url: String, params: Map<String, String>, allowErrors: Boolean = false): JSONObject {
        val body = params.entries.joinToString("&") { (k, v) -> "${enc(k)}=${enc(v)}" }.toByteArray()
        return request("POST", url, body, mapOf("Content-Type" to "application/x-www-form-urlencoded"), allowErrors)
    }

    fun request(
        method: String,
        url: String,
        body: ByteArray?,
        headers: Map<String, String>,
        allowErrors: Boolean = false,
    ): JSONObject {
        val conn = URL(url).openConnection() as HttpURLConnection
        try {
            conn.requestMethod = method
            conn.connectTimeout = 30_000
            conn.readTimeout = 120_000
            headers.forEach { (k, v) -> conn.setRequestProperty(k, v) }
            if (body != null) {
                conn.doOutput = true
                conn.setFixedLengthStreamingMode(body.size)
                conn.outputStream.use { it.write(body) }
            }
            val code = conn.responseCode
            val stream = if (code < 400) conn.inputStream else conn.errorStream
            val text = stream?.bufferedReader()?.use { it.readText() }.orEmpty()
            if (code >= 400 && !allowErrors) throw HttpException(code, text)
            return if (text.isBlank()) JSONObject() else JSONObject(text)
        } finally {
            conn.disconnect()
        }
    }

    private fun enc(s: String) = URLEncoder.encode(s, "UTF-8")
}

class HttpException(val code: Int, body: String) : Exception("HTTP $code: $body")
