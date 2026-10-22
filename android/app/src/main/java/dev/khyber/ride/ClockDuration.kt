package dev.khyber.ride

/** A duration as the rider reads one: m:ss, or h:mm:ss from an hour. */
fun clockDuration(seconds: Double): String {
    val total = seconds.toLong().coerceAtLeast(0)
    val h = total / 3600
    val m = total / 60 % 60
    val s = total % 60
    return if (h > 0) "%d:%02d:%02d".format(h, m, s) else "%d:%02d".format(m, s)
}
