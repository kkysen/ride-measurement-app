package dev.khyber.ride

import android.hardware.SensorEvent

/**
 * Sensor samples collected on the Kotlin side, to cross into Rust in one call: each call through the bindings
 * costs tens of microseconds, too much per sample at hundreds of samples a second.
 * Only collects numbers; Rust builds chunks from them (`SensorChunk::from_sample_bits`).
 * Values are kept as raw bits (`floatToRawIntBits` keeps NaN payloads), so each reaches the file exactly.
 */
class SampleBuffer(initialCapacity: Int = 1024) {
    private var timestamps = LongArray(initialCapacity)
    private var bits = IntArray(initialCapacity * 3)
    private var accuracy = IntArray(initialCapacity)
    var size = 0
        private set

    /** Values per sample of every sample so far; a sample of a different size needs a new buffer (see [fits]). */
    var valuesPerSample = 0
        private set

    val firstTimestampNs: Long get() = timestamps[0]

    fun fits(valueCount: Int) = size == 0 || valueCount == valuesPerSample

    fun add(event: SensorEvent) {
        val values = event.values
        check(fits(values.size)) { "sample has ${values.size} values, buffer has $valuesPerSample" }
        if (size == 0) valuesPerSample = values.size
        if (size == timestamps.size) {
            timestamps = timestamps.copyOf(size * 2)
            accuracy = accuracy.copyOf(size * 2)
        }
        if ((size + 1) * valuesPerSample > bits.size) bits = bits.copyOf(maxOf(bits.size * 2, (size + 1) * valuesPerSample))
        timestamps[size] = event.timestamp
        for (i in values.indices) bits[size * valuesPerSample + i] = values[i].toRawBits()
        accuracy[size] = event.accuracy
        size++
    }

    fun timestampList(): List<Long> = timestamps.asList().subList(0, size)

    fun bitList(): List<UInt> = List(size * valuesPerSample) { bits[it].toUInt() }

    fun floatList(): List<Float> = List(size * valuesPerSample) { Float.fromBits(bits[it]) }

    fun accuracyList(): List<Int> = accuracy.asList().subList(0, size)

    fun clear() {
        size = 0
        valuesPerSample = 0
    }
}
