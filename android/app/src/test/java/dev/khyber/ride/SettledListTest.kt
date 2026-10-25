package dev.khyber.ride

import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Test
import uniffi.ride_ffi.Fusion

class SettledListTest {
    @Test
    fun keepsTheSettledAndReplacesTheRest() {
        val l = SettledList<Int>()
        assertTrue(l.apply(0u, 0u, listOf(1, 2)))
        assertTrue(l.apply(0u, 1u, listOf(1, 3, 4)))
        assertEquals(1u, l.settled)
        assertFalse("the same tail again", l.apply(1u, 1u, listOf(3, 4)))
        assertTrue(l.apply(1u, 3u, listOf(3, 5, 6)))
        assertEquals(listOf(1, 3, 5, 6), l.snapshot())
        // A new fusion starts over: nothing settled there, so everything is replaced.
        assertTrue(l.apply(0u, 0u, listOf(9)))
        assertEquals(listOf(9), l.snapshot())
    }

    @Test
    fun anEmptyFusionSendsNothing() {
        Fusion().use { f ->
            val d = f.stopsSince(0u)
            assertEquals(0u, d.from)
            assertTrue(d.items.isEmpty())
            assertTrue(f.timelineSince(0u).items.isEmpty())
        }
    }
}
