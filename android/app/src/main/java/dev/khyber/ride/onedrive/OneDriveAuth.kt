package dev.khyber.ride.onedrive

import android.content.Context
import dev.khyber.ride.BuildConfig
import org.json.JSONObject

/**
 * Microsoft identity platform device code flow for personal accounts:
 * the user enters a short code at a Microsoft URL once; the refresh token is kept in app-private preferences.
 * No redirect URI or signing-certificate hash to register, just a public client with Files.ReadWrite.AppFolder.
 */
class OneDriveAuth(context: Context) {
    private val prefs = context.getSharedPreferences("onedrive", Context.MODE_PRIVATE)

    val configured get() = BuildConfig.ONEDRIVE_CLIENT_ID.isNotEmpty()
    val signedIn get() = prefs.getString(REFRESH_TOKEN, null) != null

    data class DeviceCode(val userCode: String, val verificationUri: String, val deviceCode: String, val intervalS: Long)

    fun startDeviceCode(): DeviceCode {
        val json = Http.postForm("$AUTHORITY/devicecode", mapOf("client_id" to BuildConfig.ONEDRIVE_CLIENT_ID, "scope" to SCOPE))
        return DeviceCode(
            userCode = json.getString("user_code"),
            verificationUri = json.getString("verification_uri"),
            deviceCode = json.getString("device_code"),
            intervalS = json.optLong("interval", 5),
        )
    }

    /** Polls until the user finishes signing in; blocking, call off the main thread. */
    fun awaitDeviceCode(code: DeviceCode) {
        while (true) {
            Thread.sleep(code.intervalS * 1000)
            val json = token(
                mapOf(
                    "grant_type" to "urn:ietf:params:oauth:grant-type:device_code",
                    "device_code" to code.deviceCode,
                ),
            )
            when (json.optString("error")) {
                "" -> return save(json)
                "authorization_pending", "slow_down" -> continue
                else -> throw IllegalStateException("sign-in failed: ${json.optString("error_description")}")
            }
        }
    }

    /** A fresh access token, refreshing (and rotating the refresh token) every call. */
    fun accessToken(): String {
        val refresh = checkNotNull(prefs.getString(REFRESH_TOKEN, null)) { "not signed in to OneDrive" }
        val json = token(mapOf("grant_type" to "refresh_token", "refresh_token" to refresh))
        check(!json.has("error")) { "token refresh failed: ${json.optString("error_description")}" }
        save(json)
        return json.getString("access_token")
    }

    private fun token(params: Map<String, String>): JSONObject =
        Http.postForm("$AUTHORITY/token", params + mapOf("client_id" to BuildConfig.ONEDRIVE_CLIENT_ID), allowErrors = true)

    private fun save(json: JSONObject) {
        prefs.edit().putString(REFRESH_TOKEN, json.getString("refresh_token")).commit()
    }

    companion object {
        private const val AUTHORITY = "https://login.microsoftonline.com/consumers/oauth2/v2.0"
        private const val SCOPE = "Files.ReadWrite.AppFolder offline_access"
        private const val REFRESH_TOKEN = "refresh_token"
    }
}
