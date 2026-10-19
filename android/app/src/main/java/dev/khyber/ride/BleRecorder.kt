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
import android.os.ParcelUuid
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

    private inner class Callback(val filtered: Boolean) : ScanCallback() {
        override fun onScanResult(callbackType: Int, result: ScanResult) = record(listOf(result), filtered)

        override fun onBatchScanResults(results: MutableList<ScanResult>) = record(results, filtered)

        override fun onScanFailed(errorCode: Int) {
            Log.w(TAG, "BLE scan (filtered: $filtered) failed: $errorCode")
        }
    }

    // Android pauses scans without a real filter while the screen is off (a filter with no fields doesn't count),
    // so one scan matches known beacon formats and keeps going, and another catches everything while the screen is on,
    // which shows the format of beacons to add here.
    private val filteredCallback = Callback(filtered = true)
    private val unfilteredCallback = Callback(filtered = false)

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
        scanner.startScan(BEACON_FILTERS, settings, filteredCallback)
        scanner.startScan(null, settings, unfilteredCallback)
        scanning = true
    }

    @SuppressLint("MissingPermission")
    fun stop() {
        if (!scanning) return
        try {
            adapter?.bluetoothLeScanner?.run {
                for (c in listOf(filteredCallback, unfilteredCallback)) {
                    flushPendingScanResults(c)
                    stopScan(c)
                }
            }
        } catch (e: IllegalStateException) {
            // Bluetooth was turned off meanwhile; the scan is already gone.
        }
        scanning = false
    }

    @SuppressLint("MissingPermission")
    private fun record(results: List<ScanResult>, filtered: Boolean) {
        if (results.isEmpty()) return
        val batch = BleScanResults.newBuilder().setFiltered(filtered)
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

        /** Apple's company ID: iBeacon, and AltBeacon-style beacons that reuse it. */
        private const val APPLE_COMPANY_ID = 0x004C

        /** Eddystone's 16-bit service UUID. */
        private val EDDYSTONE = ParcelUuid.fromString("0000FEAA-0000-1000-8000-00805F9B34FB")

        private val BEACON_FILTERS = listOf(
            ScanFilter.Builder().setManufacturerData(APPLE_COMPANY_ID, byteArrayOf()).build(),
            ScanFilter.Builder().setServiceData(EDDYSTONE, byteArrayOf()).build(),
        )
    }
}
