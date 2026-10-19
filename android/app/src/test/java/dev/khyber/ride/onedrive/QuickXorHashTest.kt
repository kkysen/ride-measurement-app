package dev.khyber.ride.onedrive

import java.util.Base64
import kotlin.random.Random
import org.junit.Assert.assertEquals
import org.junit.Test

class QuickXorHashTest {
    @Test
    fun emptyIsZero() {
        assertEquals("AAAAAAAAAAAAAAAAAAAAAAAAAAA=", QuickXorHash().digestBase64())
    }

    @Test
    fun matchesReferenceAcrossSplitUpdates() {
        val random = Random(1)
        for (size in listOf(1, 7, 20, 159, 160, 161, 1000, 100_003)) {
            val data = random.nextBytes(size)
            val split = QuickXorHash()
            var offset = 0
            while (offset < size) {
                val n = minOf(random.nextInt(1, 500), size - offset)
                split.update(data, offset, n)
                offset += n
            }
            assertEquals("size $size", reference(data), split.digestBase64())
        }
    }

    /** Direct port of Microsoft's published C# `QuickXorHash`, with 64-bit cells, as an independent check. */
    private fun reference(array: ByteArray): String {
        val widthInBits = 160
        val shift = 11
        val data = LongArray((widthInBits - 1) / 64 + 1)
        var vectorArrayIndex = 0
        var vectorOffset = 0
        val iterations = minOf(array.size, widthInBits)
        for (i in 0 until iterations) {
            val isLastCell = vectorArrayIndex == data.size - 1
            val bitsInVectorCell = if (isLastCell) widthInBits - 64 * (data.size - 1) else 64
            if (vectorOffset <= bitsInVectorCell - 8) {
                var j = i
                while (j < array.size) {
                    data[vectorArrayIndex] = data[vectorArrayIndex] xor ((array[j].toLong() and 0xff) shl vectorOffset)
                    j += widthInBits
                }
            } else {
                val index1 = vectorArrayIndex
                val index2 = if (isLastCell) 0 else vectorArrayIndex + 1
                val low = bitsInVectorCell - vectorOffset
                var xored = 0L
                var j = i
                while (j < array.size) {
                    xored = xored xor (array[j].toLong() and 0xff)
                    j += widthInBits
                }
                data[index1] = data[index1] xor (xored shl vectorOffset)
                data[index2] = data[index2] xor (xored ushr low)
            }
            vectorOffset += shift
            while (vectorOffset >= bitsInVectorCell) {
                vectorArrayIndex = if (isLastCell) 0 else vectorArrayIndex + 1
                vectorOffset -= bitsInVectorCell
            }
        }
        val out = ByteArray((widthInBits - 1) / 8 + 1)
        for (cell in data.indices) {
            for (b in 0 until 8) {
                val index = cell * 8 + b
                if (index < out.size) out[index] = (data[cell] ushr (8 * b)).toByte()
            }
        }
        val length = array.size.toLong()
        for (b in 0 until 8) {
            val index = widthInBits / 8 - 8 + b
            out[index] = (out[index].toInt() xor (length ushr (8 * b)).toInt()).toByte()
        }
        return Base64.getEncoder().encodeToString(out)
    }
}
