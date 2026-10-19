package dev.khyber.ride

import org.junit.Assert.assertEquals
import org.junit.Assert.assertNull
import org.junit.Test

class HealthTest {
    @Test
    fun failuresAreContainedAndCounted() {
        Health.reset()
        assertEquals(3, Health.guard("ok") { 3 })
        repeat(5) { assertNull(Health.guard<Int>("sensor x") { throw IllegalStateException("broken") }) }
        val p = Health.problems.value.getValue("sensor x")
        assertEquals(5L, p.count)
        assertEquals("IllegalStateException: broken", p.message)
        assertEquals(setOf("sensor x"), Health.problems.value.keys)
    }
}
