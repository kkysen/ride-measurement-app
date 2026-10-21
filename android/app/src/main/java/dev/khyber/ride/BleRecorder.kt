package dev.khyber.ride

import android.Manifest
import android.annotation.SuppressLint
import android.bluetooth.BluetoothAdapter
import android.bluetooth.BluetoothManager
import android.bluetooth.le.ScanCallback
import android.bluetooth.le.ScanFilter
import android.bluetooth.le.ScanResult
import android.bluetooth.le.ScanSettings
import android.content.BroadcastReceiver
import android.content.Context
import android.content.Intent
import android.content.IntentFilter
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
 * Scanning follows Bluetooth: it starts whenever Bluetooth is turned on, mid-ride too, and ends when it's turned off.
 */
class BleRecorder(private val context: Context, private val radio: RecordStreamRecorder) {
    private val adapter = context.getSystemService(BluetoothManager::class.java)?.adapter
    private var scanning = false
    private var listening = false

    private val stateReceiver = object : BroadcastReceiver() {
        override fun onReceive(context: Context, intent: Intent) {
            Health.guard("ble") {
                when (intent.getIntExtra(BluetoothAdapter.EXTRA_STATE, BluetoothAdapter.ERROR)) {
                    BluetoothAdapter.STATE_ON -> if (!scanning) startScans()
                    // Turning off ends the scans without any callback.
                    BluetoothAdapter.STATE_TURNING_OFF, BluetoothAdapter.STATE_OFF -> scanning = false
                }
            }
        }
    }

    private inner class Callback(val filtered: Boolean) : ScanCallback() {
        override fun onScanResult(callbackType: Int, result: ScanResult) {
            Health.guard("ble") { record(listOf(result), filtered) }
        }

        override fun onBatchScanResults(results: MutableList<ScanResult>) {
            Health.guard("ble") { record(results, filtered) }
        }

        override fun onScanFailed(errorCode: Int) {
            Health.report("ble scan (filtered: $filtered)", IllegalStateException("scan failed with error $errorCode"))
        }
    }

    // Android pauses scans without a real filter while the screen is off (a filter with no fields doesn't count),
    // so one scan matches known beacon formats and keeps going, and another catches everything while the screen is on,
    // which shows the format of beacons to add here.
    private val filteredCallback = Callback(filtered = true)
    private val unfilteredCallback = Callback(filtered = false)

    fun start() {
        Health.guard("ble") {
            context.registerReceiver(stateReceiver, IntentFilter(BluetoothAdapter.ACTION_STATE_CHANGED))
            listening = true
        }
        Health.guard("ble") { startScans() }
    }

    @SuppressLint("MissingPermission")
    private fun startScans() {
        if (context.checkSelfPermission(Manifest.permission.BLUETOOTH_SCAN) != PackageManager.PERMISSION_GRANTED) {
            Health.report("ble", IllegalStateException("no Nearby devices permission: not recording beacons"))
            return
        }
        val scanner = adapter?.takeIf { it.isEnabled }?.bluetoothLeScanner ?: run {
            Health.report("ble", IllegalStateException("Bluetooth is off: not recording beacons (recording starts when it's turned on)"))
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
        if (listening) {
            Health.guard("stop ble") { context.unregisterReceiver(stateReceiver) }
            listening = false
        }
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
        } catch (e: Throwable) {
            Health.report("stop ble", e)
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
