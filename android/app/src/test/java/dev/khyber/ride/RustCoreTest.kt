package dev.khyber.ride

import org.junit.Assert.assertEquals
import org.junit.Test
import uniffi.ride_ffi.coreVersion

class RustCoreTest {
    @Test
    fun callsIntoRust() {
        assertEquals("0.1.0", coreVersion())
    }
}
