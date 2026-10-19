package dev.khyber.ride

import android.annotation.SuppressLint
import android.location.GnssAntennaInfo
import android.location.GnssMeasurementRequest
import android.location.GnssMeasurementsEvent
import android.location.GnssNavigationMessage
import android.location.GnssStatus
import android.location.Location
import android.location.LocationListener
import android.location.LocationManager
import android.os.Build
import android.os.SystemClock
import android.util.Log
import com.google.protobuf.ByteString
import dev.khyber.ride.proto.GnssAntennaInfo as PbGnssAntennaInfo
import dev.khyber.ride.proto.GnssAntennaInfos
import dev.khyber.ride.proto.GnssCapabilities as PbGnssCapabilities
import dev.khyber.ride.proto.GnssClock as PbGnssClock
import dev.khyber.ride.proto.GnssMeasurement as PbGnssMeasurement
import dev.khyber.ride.proto.GnssMeasurementsEvent as PbGnssMeasurementsEvent
import dev.khyber.ride.proto.GnssNavigationMessage as PbGnssNavigationMessage
import dev.khyber.ride.proto.GnssSatellite as PbGnssSatellite
import dev.khyber.ride.proto.GnssStatus as PbGnssStatus
import dev.khyber.ride.proto.Location as PbLocation
import dev.khyber.ride.proto.SphericalCorrections
import java.util.concurrent.Executor

/**
 * Raw GNSS (`gnss` stream) and location fixes (`location` stream).
 * Full tracking is requested, which disables duty cycling like the developer option does.
 */
class GnssRecorder(
    private val locationManager: LocationManager,
    private val gnss: RecordStreamRecorder,
    private val locations: RecordStreamRecorder,
) {
    /** `elapsedRealtimeNanos` of the last GPS fix, to tell when GNSS is lost (e.g. underground). */
    @Volatile
    var lastGpsFixElapsedNs = 0L
        private set

    private val measurementsCallback = object : GnssMeasurementsEvent.Callback() {
        override fun onGnssMeasurementsReceived(event: GnssMeasurementsEvent) {
            gnss.add { gnssMeasurements = event.toProto() }
        }
    }

    private val statusCallback = object : GnssStatus.Callback() {
        override fun onSatelliteStatusChanged(status: GnssStatus) {
            gnss.add { gnssStatus = status.toProto() }
        }
    }

    private val navigationCallback = object : GnssNavigationMessage.Callback() {
        override fun onGnssNavigationMessageReceived(message: GnssNavigationMessage) {
            gnss.add { gnssNavigationMessage = message.toProto() }
        }
    }

    private val gpsListener = LocationListener { location ->
        lastGpsFixElapsedNs = SystemClock.elapsedRealtimeNanos()
        locations.add { this.location = location.toProto() }
        LiveSpeed.onGpsLocation(location)
        LiveTrack.onGpsLocation(location)
    }

    private val antennaListener = GnssAntennaInfo.Listener { infos ->
        gnss.add { gnssAntennaInfos = GnssAntennaInfos.newBuilder().addAllAntennas(infos.map { it.toProto() }).build() }
    }

    /** Every boolean `has*` capability by name, so new Android capabilities are recorded without code changes. */
    private fun capabilitiesProto(): PbGnssCapabilities {
        val caps = locationManager.gnssCapabilities
        val b = PbGnssCapabilities.newBuilder().setYearOfHardware(locationManager.gnssYearOfHardware)
        locationManager.gnssHardwareModelName?.let { b.hardwareModelName = it }
        for (m in caps.javaClass.methods) {
            if (m.name.startsWith("has") && m.parameterCount == 0 && m.returnType == Boolean::class.javaPrimitiveType) {
                try {
                    b.putFlags(m.name.removePrefix("has"), m.invoke(caps) as Boolean)
                } catch (e: ReflectiveOperationException) {
                    Log.w("GnssRecorder", "capability ${m.name}", e)
                }
            }
        }
        return b.build()
    }

    private val fusedListener = LocationListener { location -> locations.add { this.location = location.toProto() } }

    @SuppressLint("MissingPermission")
    fun start() {
        val executor = Executor { gnss.handler.post(it) }
        locationManager.registerGnssMeasurementsCallback(
            GnssMeasurementRequest.Builder().setFullTracking(true).build(),
            executor,
            measurementsCallback,
        )
        locationManager.registerGnssStatusCallback(executor, statusCallback)
        locationManager.registerGnssNavigationMessageCallback(executor, navigationCallback)
        gnss.add { gnssCapabilities = capabilitiesProto() }
        // Phase center offsets and corrections; phones without antenna info just never call back.
        if (!locationManager.registerAntennaInfoListener(executor, antennaListener)) {
            Log.i("GnssRecorder", "no GNSS antenna info on this phone")
        }
        val looper = locations.handler.looper
        locationManager.requestLocationUpdates(LocationManager.GPS_PROVIDER, 0L, 0f, gpsListener, looper)
        if (locationManager.hasProvider(LocationManager.FUSED_PROVIDER)) {
            locationManager.requestLocationUpdates(LocationManager.FUSED_PROVIDER, 1_000L, 0f, fusedListener, looper)
        }
    }

    fun stop() {
        locationManager.unregisterGnssMeasurementsCallback(measurementsCallback)
        locationManager.unregisterGnssStatusCallback(statusCallback)
        locationManager.unregisterGnssNavigationMessageCallback(navigationCallback)
        locationManager.unregisterAntennaInfoListener(antennaListener)
        locationManager.removeUpdates(gpsListener)
        locationManager.removeUpdates(fusedListener)
    }
}

private fun GnssMeasurementsEvent.toProto(): PbGnssMeasurementsEvent {
    val b = PbGnssMeasurementsEvent.newBuilder().setClock(clock.toProto())
    measurements.forEach { b.addMeasurements(it.toProto()) }
    if (Build.VERSION.SDK_INT >= 34 && hasIsFullTracking()) b.isFullTracking = isFullTracking
    return b.build()
}

private fun android.location.GnssClock.toProto(): PbGnssClock {
    val b = PbGnssClock.newBuilder()
        .setTimeNanos(timeNanos)
        .setHardwareClockDiscontinuityCount(hardwareClockDiscontinuityCount)
    if (hasTimeUncertaintyNanos()) b.timeUncertaintyNanos = timeUncertaintyNanos
    if (hasLeapSecond()) b.leapSecond = leapSecond
    if (hasFullBiasNanos()) b.fullBiasNanos = fullBiasNanos
    if (hasBiasNanos()) b.biasNanos = biasNanos
    if (hasBiasUncertaintyNanos()) b.biasUncertaintyNanos = biasUncertaintyNanos
    if (hasDriftNanosPerSecond()) b.driftNanosPerSecond = driftNanosPerSecond
    if (hasDriftUncertaintyNanosPerSecond()) b.driftUncertaintyNanosPerSecond = driftUncertaintyNanosPerSecond
    if (hasElapsedRealtimeNanos()) b.elapsedRealtimeNanos = elapsedRealtimeNanos
    if (hasElapsedRealtimeUncertaintyNanos()) b.elapsedRealtimeUncertaintyNanos = elapsedRealtimeUncertaintyNanos
    if (hasReferenceCarrierFrequencyHzForIsb()) b.referenceCarrierFrequencyHzForIsb = referenceCarrierFrequencyHzForIsb
    if (hasReferenceCodeTypeForIsb()) b.referenceCodeTypeForIsb = referenceCodeTypeForIsb
    if (hasReferenceConstellationTypeForIsb()) b.referenceConstellationTypeForIsb = referenceConstellationTypeForIsb
    return b.build()
}

@Suppress("DEPRECATION") // `snrInDb` and `automaticGainControlLevelDb` are deprecated but still raw data.
private fun android.location.GnssMeasurement.toProto(): PbGnssMeasurement {
    val b = PbGnssMeasurement.newBuilder()
        .setSvid(svid)
        .setConstellationType(constellationType)
        .setTimeOffsetNanos(timeOffsetNanos)
        .setState(state)
        .setReceivedSvTimeNanos(receivedSvTimeNanos)
        .setReceivedSvTimeUncertaintyNanos(receivedSvTimeUncertaintyNanos)
        .setCn0Dbhz(cn0DbHz)
        .setPseudorangeRateMetersPerSecond(pseudorangeRateMetersPerSecond)
        .setPseudorangeRateUncertaintyMetersPerSecond(pseudorangeRateUncertaintyMetersPerSecond)
        .setAccumulatedDeltaRangeState(accumulatedDeltaRangeState)
        .setAccumulatedDeltaRangeMeters(accumulatedDeltaRangeMeters)
        .setAccumulatedDeltaRangeUncertaintyMeters(accumulatedDeltaRangeUncertaintyMeters)
        .setMultipathIndicator(multipathIndicator)
    if (hasBasebandCn0DbHz()) b.basebandCn0Dbhz = basebandCn0DbHz
    if (hasCarrierFrequencyHz()) b.carrierFrequencyHz = carrierFrequencyHz
    if (hasSnrInDb()) b.snrInDb = snrInDb
    if (hasAutomaticGainControlLevelDb()) b.automaticGainControlLevelDb = automaticGainControlLevelDb
    if (hasCodeType()) b.codeType = codeType
    if (hasFullInterSignalBiasNanos()) b.fullInterSignalBiasNanos = fullInterSignalBiasNanos
    if (hasFullInterSignalBiasUncertaintyNanos()) b.fullInterSignalBiasUncertaintyNanos = fullInterSignalBiasUncertaintyNanos
    if (hasSatelliteInterSignalBiasNanos()) b.satelliteInterSignalBiasNanos = satelliteInterSignalBiasNanos
    if (hasSatelliteInterSignalBiasUncertaintyNanos()) {
        b.satelliteInterSignalBiasUncertaintyNanos = satelliteInterSignalBiasUncertaintyNanos
    }
    return b.build()
}

private fun GnssStatus.toProto(): PbGnssStatus {
    val b = PbGnssStatus.newBuilder()
    for (i in 0 until satelliteCount) {
        val s = PbGnssSatellite.newBuilder()
            .setSvid(getSvid(i))
            .setConstellationType(getConstellationType(i))
            .setCn0Dbhz(getCn0DbHz(i))
            .setElevationDegrees(getElevationDegrees(i))
            .setAzimuthDegrees(getAzimuthDegrees(i))
            .setHasAlmanac(hasAlmanacData(i))
            .setHasEphemeris(hasEphemerisData(i))
            .setUsedInFix(usedInFix(i))
        if (hasCarrierFrequencyHz(i)) s.carrierFrequencyHz = getCarrierFrequencyHz(i)
        if (hasBasebandCn0DbHz(i)) s.basebandCn0Dbhz = getBasebandCn0DbHz(i)
        b.addSatellites(s)
    }
    return b.build()
}

private fun GnssNavigationMessage.toProto(): PbGnssNavigationMessage = PbGnssNavigationMessage.newBuilder()
    .setSvid(svid)
    .setType(type)
    .setStatus(status)
    .setMessageId(messageId)
    .setSubmessageId(submessageId)
    .setData(ByteString.copyFrom(data))
    .build()

private fun Location.toProto(): PbLocation {
    val b = PbLocation.newBuilder()
        .setProvider(provider.orEmpty())
        .setTimeMs(time)
        .setElapsedRealtimeNanos(elapsedRealtimeNanos)
        .setLatitudeDegrees(latitude)
        .setLongitudeDegrees(longitude)
        .setIsMock(isMock)
    if (hasElapsedRealtimeUncertaintyNanos()) b.elapsedRealtimeUncertaintyNanos = elapsedRealtimeUncertaintyNanos
    if (hasAltitude()) b.altitudeMeters = altitude
    if (Build.VERSION.SDK_INT >= 34) {
        if (hasMslAltitude()) b.mslAltitudeMeters = mslAltitudeMeters
        if (hasMslAltitudeAccuracy()) b.mslAltitudeAccuracyMeters = mslAltitudeAccuracyMeters
    }
    if (hasSpeed()) b.speedMetersPerSecond = speed
    if (hasBearing()) b.bearingDegrees = bearing
    if (hasAccuracy()) b.horizontalAccuracyMeters = accuracy
    if (hasVerticalAccuracy()) b.verticalAccuracyMeters = verticalAccuracyMeters
    if (hasSpeedAccuracy()) b.speedAccuracyMetersPerSecond = speedAccuracyMetersPerSecond
    if (hasBearingAccuracy()) b.bearingAccuracyDegrees = bearingAccuracyDegrees
    return b.build()
}

private fun GnssAntennaInfo.toProto(): PbGnssAntennaInfo {
    val o = phaseCenterOffset
    val b = PbGnssAntennaInfo.newBuilder()
        .setCarrierFrequencyMhz(carrierFrequencyMHz)
        .setPhaseCenterOffsetXMm(o.xOffsetMm)
        .setPhaseCenterOffsetXUncertaintyMm(o.xOffsetUncertaintyMm)
        .setPhaseCenterOffsetYMm(o.yOffsetMm)
        .setPhaseCenterOffsetYUncertaintyMm(o.yOffsetUncertaintyMm)
        .setPhaseCenterOffsetZMm(o.zOffsetMm)
        .setPhaseCenterOffsetZUncertaintyMm(o.zOffsetUncertaintyMm)
    phaseCenterVariationCorrections?.let { b.phaseCenterVariationCorrections = it.toProto() }
    signalGainCorrections?.let { b.signalGainCorrections = it.toProto() }
    return b.build()
}

private fun GnssAntennaInfo.SphericalCorrections.toProto(): SphericalCorrections {
    val values = correctionsArray
    val uncertainties = correctionUncertaintiesArray
    val b = SphericalCorrections.newBuilder()
        .setRows(values.size)
        .setColumns(values.firstOrNull()?.size ?: 0)
        .setDeltaThetaDegrees(deltaTheta)
        .setDeltaPhiDegrees(deltaPhi)
    values.forEach { row -> row.forEach { b.addCorrections(it) } }
    uncertainties.forEach { row -> row.forEach { b.addUncertainties(it) } }
    return b.build()
}
