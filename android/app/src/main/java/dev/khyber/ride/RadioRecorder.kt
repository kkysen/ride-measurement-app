package dev.khyber.ride

import android.annotation.SuppressLint
import android.content.BroadcastReceiver
import android.content.Context
import android.content.Intent
import android.content.IntentFilter
import android.net.wifi.WifiManager
import android.os.BatteryManager
import android.os.SystemClock
import android.telephony.CellInfo
import android.telephony.CellInfoGsm
import android.telephony.CellInfoLte
import android.telephony.CellInfoNr
import android.telephony.CellInfoTdscdma
import android.telephony.CellInfoWcdma
import android.telephony.CellIdentityNr
import android.telephony.CellSignalStrength
import android.telephony.ServiceState
import android.telephony.SignalStrength
import android.telephony.TelephonyCallback
import android.telephony.TelephonyManager
import dev.khyber.ride.proto.BatteryStatus
import java.util.concurrent.Executor
import dev.khyber.ride.proto.Cell
import dev.khyber.ride.proto.CellInfoSnapshot
import dev.khyber.ride.proto.ServiceState as PbServiceState
import dev.khyber.ride.proto.SignalStrength as PbSignalStrength
import dev.khyber.ride.proto.WifiResult
import dev.khyber.ride.proto.WifiScan

/**
 * Cell info (polled; passive, the modem doesn't search), service state, signal strength, WiFi scans, and battery,
 * all into the `radio` stream.
 *
 * WiFi scans are requested only while GNSS is lost, since Android allows 4 scans per 2 minutes;
 * results from scans anyone else triggers are always recorded, for free.
 * None of this is ever used as a position directly, only as fingerprints.
 */
class RadioRecorder(
    private val context: Context,
    private val radio: RecordStreamRecorder,
    /** `elapsedRealtimeNanos` of the last GPS fix. */
    private val lastGpsFixElapsedNs: () -> Long,
) {
    private val telephony = context.getSystemService(TelephonyManager::class.java)
    private val wifi = context.getSystemService(WifiManager::class.java)
    private val battery = context.getSystemService(BatteryManager::class.java)
    private val handler = radio.handler
    private var lastScanRequestNs = 0L

    @Volatile
    private var scanRequested = false
    private var telephonyRegistered = false

    private val telephonyCallback = object :
        TelephonyCallback(),
        TelephonyCallback.ServiceStateListener,
        TelephonyCallback.SignalStrengthsListener {
        @SuppressLint("MissingPermission")
        override fun onServiceStateChanged(state: ServiceState) {
            val operator = try {
                state.operatorNumeric.orEmpty()
            } catch (e: SecurityException) {
                ""
            }
            radio.add {
                serviceState = PbServiceState.newBuilder()
                    .setState(state.state)
                    .setRoaming(state.roaming)
                    .setOperatorNumeric(operator)
                    .setRaw(state.toString())
                    .build()
            }
        }

        override fun onSignalStrengthsChanged(strength: SignalStrength) {
            radio.add {
                signalStrength = PbSignalStrength.newBuilder().setLevel(strength.level).setRaw(strength.toString()).build()
            }
        }
    }

    private val wifiReceiver = object : BroadcastReceiver() {
        @SuppressLint("MissingPermission")
        override fun onReceive(context: Context, intent: Intent) {
            val requested = scanRequested
            scanRequested = false
            val results = wifi.scanResults
            radio.add { wifiScan = WifiScan.newBuilder().setRequested(requested).addAllResults(results.map { it.toProto() }).build() }
        }
    }

    private val batteryReceiver = object : BroadcastReceiver() {
        override fun onReceive(context: Context, intent: Intent) = recordBattery(intent)
    }

    private val poll = object : Runnable {
        override fun run() {
            requestCellInfo()
            maybeScanWifi()
            recordBattery(null)
            handler.postDelayed(this, POLL_MS)
        }
    }

    @SuppressLint("MissingPermission")
    fun start() {
        try {
            telephony.registerTelephonyCallback(Executor { handler.post(it) }, telephonyCallback)
            telephonyRegistered = true
        } catch (e: SecurityException) {
            android.util.Log.w("RadioRecorder", "no phone state permission; not recording service state", e)
        }
        context.registerReceiver(wifiReceiver, IntentFilter(WifiManager.SCAN_RESULTS_AVAILABLE_ACTION), null, handler)
        context.registerReceiver(batteryReceiver, IntentFilter(Intent.ACTION_BATTERY_CHANGED), null, handler)
        handler.post(poll)
    }

    fun stop() {
        handler.removeCallbacks(poll)
        if (telephonyRegistered) telephony.unregisterTelephonyCallback(telephonyCallback)
        context.unregisterReceiver(wifiReceiver)
        context.unregisterReceiver(batteryReceiver)
    }

    @SuppressLint("MissingPermission")
    private fun requestCellInfo() {
        try {
            telephony.requestCellInfoUpdate(
                Executor { handler.post(it) },
                object : TelephonyManager.CellInfoCallback() {
                    override fun onCellInfo(cells: MutableList<CellInfo>) {
                        radio.add { cellInfo = CellInfoSnapshot.newBuilder().addAllCells(cells.map { it.toProto() }).build() }
                    }
                },
            )
        } catch (e: SecurityException) {
            android.util.Log.w("RadioRecorder", "no location permission; not recording cell info", e)
        }
    }

    @Suppress("DEPRECATION") // `startScan` is deprecated but still the only way to request a scan.
    private fun maybeScanWifi() {
        val now = SystemClock.elapsedRealtimeNanos()
        val gnssLost = now - lastGpsFixElapsedNs() > GNSS_LOST_NS
        if (!gnssLost || now - lastScanRequestNs < SCAN_INTERVAL_NS) return
        lastScanRequestNs = now
        scanRequested = wifi.startScan()
    }

    private fun recordBattery(sticky: Intent?) {
        val intent = sticky ?: context.registerReceiver(null, IntentFilter(Intent.ACTION_BATTERY_CHANGED)) ?: return
        val b = BatteryStatus.newBuilder()
            .setLevel(intent.getIntExtra(BatteryManager.EXTRA_LEVEL, -1))
            .setScale(intent.getIntExtra(BatteryManager.EXTRA_SCALE, -1))
            .setStatus(intent.getIntExtra(BatteryManager.EXTRA_STATUS, -1))
            .setPlugged(intent.getIntExtra(BatteryManager.EXTRA_PLUGGED, -1))
            .setTemperatureDecidegreesC(intent.getIntExtra(BatteryManager.EXTRA_TEMPERATURE, Int.MIN_VALUE))
            .setVoltageMv(intent.getIntExtra(BatteryManager.EXTRA_VOLTAGE, -1))
        val current = battery.getIntProperty(BatteryManager.BATTERY_PROPERTY_CURRENT_NOW)
        if (current != Int.MIN_VALUE) b.currentNowUa = current
        val charge = battery.getIntProperty(BatteryManager.BATTERY_PROPERTY_CHARGE_COUNTER)
        if (charge != Int.MIN_VALUE) b.chargeCounterUah = charge
        radio.add { this.battery = b.build() }
    }

    companion object {
        private const val POLL_MS = 5_000L
        private const val GNSS_LOST_NS = 10_000_000_000L
        private const val SCAN_INTERVAL_NS = 30_000_000_000L
    }
}

private fun CellInfo.toProto(): Cell {
    val b = Cell.newBuilder()
        .setRegistered(isRegistered)
        .setConnectionStatus(cellConnectionStatus)
        .setTimestampMs(timestampMillis)
        .setRaw(toString())
    val signal: CellSignalStrength = cellSignalStrength
    b.setDbm(signal.dbm).setAsuLevel(signal.asuLevel).setLevel(signal.level)
    when (this) {
        is CellInfoLte -> cellIdentity.run {
            b.kind = "lte"
            b.mcc = mccString.orEmpty()
            b.mnc = mncString.orEmpty()
            if (ci != CellInfo.UNAVAILABLE) b.cellId = ci.toLong()
            if (pci != CellInfo.UNAVAILABLE) b.pci = pci
            if (tac != CellInfo.UNAVAILABLE) b.tac = tac
            if (earfcn != CellInfo.UNAVAILABLE) b.arfcn = earfcn
            b.addAllBands(bands.toList())
        }
        is CellInfoNr -> (cellIdentity as CellIdentityNr).run {
            b.kind = "nr"
            b.mcc = mccString.orEmpty()
            b.mnc = mncString.orEmpty()
            if (nci != CellInfo.UNAVAILABLE_LONG) b.cellId = nci
            if (pci != CellInfo.UNAVAILABLE) b.pci = pci
            if (tac != CellInfo.UNAVAILABLE) b.tac = tac
            if (nrarfcn != CellInfo.UNAVAILABLE) b.arfcn = nrarfcn
            b.addAllBands(bands.toList())
        }
        is CellInfoGsm -> cellIdentity.run {
            b.kind = "gsm"
            b.mcc = mccString.orEmpty()
            b.mnc = mncString.orEmpty()
            if (cid != CellInfo.UNAVAILABLE) b.cellId = cid.toLong()
            if (lac != CellInfo.UNAVAILABLE) b.tac = lac
            if (arfcn != CellInfo.UNAVAILABLE) b.arfcn = arfcn
        }
        is CellInfoWcdma -> cellIdentity.run {
            b.kind = "wcdma"
            b.mcc = mccString.orEmpty()
            b.mnc = mncString.orEmpty()
            if (cid != CellInfo.UNAVAILABLE) b.cellId = cid.toLong()
            if (psc != CellInfo.UNAVAILABLE) b.pci = psc
            if (lac != CellInfo.UNAVAILABLE) b.tac = lac
            if (uarfcn != CellInfo.UNAVAILABLE) b.arfcn = uarfcn
        }
        is CellInfoTdscdma -> cellIdentity.run {
            b.kind = "tdscdma"
            b.mcc = mccString.orEmpty()
            b.mnc = mncString.orEmpty()
            if (cid != CellInfo.UNAVAILABLE) b.cellId = cid.toLong()
            if (lac != CellInfo.UNAVAILABLE) b.tac = lac
            if (uarfcn != CellInfo.UNAVAILABLE) b.arfcn = uarfcn
        }
        else -> b.kind = javaClass.simpleName
    }
    return b.build()
}

private fun android.net.wifi.ScanResult.toProto(): WifiResult = WifiResult.newBuilder()
    .setBssid(BSSID.orEmpty())
    .setSsid(
        if (android.os.Build.VERSION.SDK_INT >= 33) {
            wifiSsid?.toString().orEmpty()
        } else {
            @Suppress("DEPRECATION")
            SSID.orEmpty()
        },
    )
    .setFrequencyMhz(frequency)
    .setLevelDbm(level)
    .setTimestampUs(timestamp)
    .setChannelWidth(channelWidth)
    .setWifiStandard(wifiStandard)
    .build()
