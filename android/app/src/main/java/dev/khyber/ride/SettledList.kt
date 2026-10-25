package dev.khyber.ride

/**
 * The app's copy of a list Rust sends as deltas (`Fusion.stopsSince`, `Fusion.timelineSince`):
 * the first [settled] items never change, so each tick only the rest crosses, however long the ride.
 */
class SettledList<T> {
    private val items = ArrayList<T>()

    /** How many items are settled: ask Rust from here next. */
    var settled: UInt = 0u
        private set

    /** Keeps the first [from] items and replaces the rest with [tail]; whether anything changed. */
    fun apply(from: UInt, settled: UInt, tail: List<T>): Boolean {
        this.settled = settled
        val keep = minOf(from.toInt(), items.size)
        val rest = items.subList(keep, items.size)
        if (rest == tail) return false
        rest.clear()
        items.addAll(tail)
        return true
    }

    fun snapshot(): List<T> = items.toList()

    fun clear() {
        items.clear()
        settled = 0u
    }
}
