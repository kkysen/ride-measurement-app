package dev.kkysen.ride

import android.Manifest
import android.content.Intent
import android.net.Uri
import android.os.Bundle
import android.os.Environment
import android.os.PowerManager
import android.provider.Settings
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import androidx.activity.result.contract.ActivityResultContracts
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.padding
import androidx.compose.material3.Button
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Surface
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.rememberCoroutineScope
import androidx.compose.runtime.setValue
import androidx.compose.ui.Modifier
import androidx.compose.ui.unit.dp
import androidx.lifecycle.compose.collectAsStateWithLifecycle
import dev.kkysen.ride.onedrive.OneDriveAuth
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.launch
import kotlinx.coroutines.withContext

class MainActivity : ComponentActivity() {
    private val permissions = registerForActivityResult(ActivityResultContracts.RequestMultiplePermissions()) {}

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        permissions.launch(
            arrayOf(
                Manifest.permission.ACCESS_FINE_LOCATION,
                Manifest.permission.ACCESS_COARSE_LOCATION,
                Manifest.permission.POST_NOTIFICATIONS,
            ),
        )
        setContent {
            MaterialTheme {
                Surface(Modifier.fillMaxSize()) { Screen() }
            }
        }
    }

    @Composable
    private fun Screen() {
        val state by RecorderService.state.collectAsStateWithLifecycle()
        val auth = remember { OneDriveAuth(this) }
        var oneDriveStatus by remember { mutableStateOf(if (auth.signedIn) "OneDrive: signed in" else "OneDrive: not signed in") }
        val scope = rememberCoroutineScope()

        Column(Modifier.padding(24.dp), verticalArrangement = Arrangement.spacedBy(16.dp)) {
            if (!Environment.isExternalStorageManager()) {
                Text("Recordings are kept in shared storage so an uninstall can't delete them; this needs all-files access.")
                Button(onClick = {
                    startActivity(
                        Intent(Settings.ACTION_MANAGE_APP_ALL_FILES_ACCESS_PERMISSION, Uri.parse("package:$packageName")),
                    )
                }) { Text("Grant all-files access") }
                return@Column
            }

            if (!getSystemService(PowerManager::class.java).isIgnoringBatteryOptimizations(packageName)) {
                Text("Battery optimization can stop recording mid-ride; exempting this app makes that much less likely.")
                Button(onClick = {
                    startActivity(
                        Intent(Settings.ACTION_REQUEST_IGNORE_BATTERY_OPTIMIZATIONS, Uri.parse("package:$packageName")),
                    )
                }) { Text("Don't optimize battery for this app") }
            }

            Text(if (state.recording) "Recording ${state.sessionId}" else "Not recording")
            Button(onClick = {
                if (state.recording) RecorderService.stop(this@MainActivity) else RecorderService.start(this@MainActivity)
            }) { Text(if (state.recording) "Stop" else "Start") }

            Text(if (auth.configured) oneDriveStatus else "OneDrive: no client ID in this build")
            if (auth.configured) {
                Button(onClick = {
                    scope.launch {
                        try {
                            val code = withContext(Dispatchers.IO) { auth.startDeviceCode() }
                            oneDriveStatus = "Go to ${code.verificationUri} and enter ${code.userCode}"
                            withContext(Dispatchers.IO) { auth.awaitDeviceCode(code) }
                            oneDriveStatus = "OneDrive: signed in"
                            UploadWorker.enqueue(this@MainActivity)
                        } catch (e: Exception) {
                            oneDriveStatus = "OneDrive sign-in failed: ${e.message}"
                        }
                    }
                }) { Text(if (auth.signedIn) "Sign in again" else "Sign in to OneDrive") }
                Button(onClick = { UploadWorker.enqueue(this@MainActivity) }) { Text("Upload now") }
            }
        }
    }
}
