package dev.kkysen.ride.onedrive

import java.util.Base64

/**
 * OneDrive's `quickXorHash`: each input byte is XORed into a 160-bit circular register
 * at bit offset `11 * index mod 160`, then the total length is XORed into the last 8 bytes (little-endian).
 */
class QuickXorHash {
    private val register = ByteArray(WIDTH_BITS / 8)
    private var length = 0L

    fun update(data: ByteArray, offset: Int = 0, count: Int = data.size) {
        for (i in offset until offset + count) {
            val bit = ((length % WIDTH_BITS) * SHIFT % WIDTH_BITS).toInt()
            val b = data[i].toInt() and 0xff
            val byteIndex = bit / 8
            val bitInByte = bit % 8
            register[byteIndex] = (register[byteIndex].toInt() xor (b shl bitInByte)).toByte()
            if (bitInByte != 0) {
                val next = (byteIndex + 1) % register.size
                register[next] = (register[next].toInt() xor (b ushr (8 - bitInByte))).toByte()
            }
            length++
        }
    }

    fun digest(): ByteArray {
        val out = register.copyOf()
        for (i in 0 until 8) {
            val index = out.size - 8 + i
            out[index] = (out[index].toInt() xor (length ushr (8 * i)).toInt()).toByte()
        }
        return out
    }

    fun digestBase64(): String = Base64.getEncoder().encodeToString(digest())

    companion object {
        private const val WIDTH_BITS = 160
        private const val SHIFT = 11L
    }
}
