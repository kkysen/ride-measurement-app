package dev.khyber.ride

import android.Manifest
import android.annotation.SuppressLint
import android.bluetooth.BluetoothManager
import android.bluetooth.le.ScanCallback
import android.bluetooth.le.ScanFilter
import android.bluetooth.le.ScanResult
import android.bluetooth.le.ScanSettings
import android.content.Context
import android.content.pm.PackageManager
import android.util.Log
import com.google.protobuf.ByteString
import dev.khyber.ride.proto.BleScanResult
import dev.khyber.ride.proto.BleScanResults

/**
 * Bluetooth LE advertisements around the phone, into the radio stream: beacons in stations and trains
 * (e.g. Piper beacons on B Division subway cars) are fingerprints for where the phone is, underground too.
 * Every advertisement is kept whole, so any beacon format can be decoded later.
 */
class BleRecorder(private val context: Context, private val radio: RecordStreamRecorder) {
    private val adapter = context.getSystemService(BluetoothManager::class.java)?.adapter
    private var scanning = false

    private val callback = object : ScanCallback() {
        override fun onScanResult(callbackType: Int, result: ScanResult) = record(listOf(result))

        override fun onBatchScanResults(results: MutableList<ScanResult>) = record(results)

        override fun onScanFailed(errorCode: Int) {
            Log.w(TAG, "BLE scan failed: $errorCode")
        }
    }

    @SuppressLint("MissingPermission")
    fun start() {
        if (context.checkSelfPermission(Manifest.permission.BLUETOOTH_SCAN) != PackageManager.PERMISSION_GRANTED) {
            Log.i(TAG, "no Bluetooth scan permission; not recording BLE")
            return
        }
        val scanner = adapter?.takeIf { it.isEnabled }?.bluetoothLeScanner ?: run {
            Log.i(TAG, "Bluetooth off or missing; not recording BLE")
            return
        }
        val settings = ScanSettings.Builder()
            .setScanMode(ScanSettings.SCAN_MODE_BALANCED)
            .setCallbackType(ScanSettings.CALLBACK_TYPE_ALL_MATCHES)
            .setLegacy(false)
            // Batch in the controller where it can, to save power; results keep their own timestamps.
            .apply { if (adapter.isOffloadedScanBatchingSupported) setReportDelay(REPORT_DELAY_MS) }
            .build()
        // Android pauses scans without a filter while the screen is off; an empty filter still matches everything.
        scanner.startScan(listOf(ScanFilter.Builder().build()), settings, callback)
        scanning = true
    }

    @SuppressLint("MissingPermission")
    fun stop() {
        if (!scanning) return
        try {
            adapter?.bluetoothLeScanner?.run {
                flushPendingScanResults(callback)
                stopScan(callback)
            }
        } catch (e: IllegalStateException) {
            // Bluetooth was turned off meanwhile; the scan is already gone.
        }
        scanning = false
    }

    @SuppressLint("MissingPermission")
    private fun record(results: List<ScanResult>) {
        if (results.isEmpty()) return
        val batch = BleScanResults.newBuilder()
        for (r in results) {
            val b = BleScanResult.newBuilder()
                .setAddress(r.device.address)
                .setRssi(r.rssi)
                .setTimestampElapsedRealtimeNs(r.timestampNanos)
                .setPrimaryPhy(r.primaryPhy)
                .setSecondaryPhy(r.secondaryPhy)
                .setAdvertisingSid(r.advertisingSid)
                .setPeriodicAdvertisingInterval(r.periodicAdvertisingInterval)
                .setConnectable(r.isConnectable)
                .setDataStatus(r.dataStatus)
            r.scanRecord?.bytes?.let { b.advertisingData = ByteString.copyFrom(it) }
            if (r.txPower != ScanResult.TX_POWER_NOT_PRESENT) b.txPower = r.txPower
            batch.addResults(b)
        }
        radio.add { bleScan = batch.build() }
    }

    companion object {
        private const val TAG = "BleRecorder"
        private const val REPORT_DELAY_MS = 2_000L
    }
}
