package dev.khyber.ride

import android.app.Notification
import android.app.NotificationChannel
import android.app.NotificationManager
import android.app.PendingIntent
import android.content.Context
import android.content.Intent

/**
 * Reminds the rider that it's still recording once they've stayed in one place, out of the subway,
 * for [AFTER_S] (Rust's `Status::staying_s`: home, a restaurant), once per stay.
 */
class StayReminder(private val context: Context) {
    private var reminded = false

    fun check(stayingS: Double) {
        if (stayingS < AFTER_S) {
            if (reminded) cancel()
            reminded = false
            return
        }
        if (reminded) return
        reminded = true
        val manager = context.getSystemService(NotificationManager::class.java)
        manager.createNotificationChannel(NotificationChannel(CHANNEL_ID, "Still recording", NotificationManager.IMPORTANCE_DEFAULT))
        val open = PendingIntent.getActivity(
            context,
            0,
            Intent(context, MainActivity::class.java).addFlags(Intent.FLAG_ACTIVITY_SINGLE_TOP),
            PendingIntent.FLAG_IMMUTABLE,
        )
        manager.notify(
            NOTIFICATION_ID,
            Notification.Builder(context, CHANNEL_ID)
                .setContentTitle("Still recording")
                .setContentText("You've been in one place for %.0f minutes, not in the subway. Stop the recording?".format(stayingS / 60))
                .setSmallIcon(android.R.drawable.ic_dialog_info)
                .setContentIntent(open)
                .setAutoCancel(true)
                .build(),
        )
    }

    fun cancel() {
        context.getSystemService(NotificationManager::class.java).cancel(NOTIFICATION_ID)
    }

    companion object {
        const val AFTER_S = 10 * 60.0
        private const val CHANNEL_ID = "still-recording"
        private const val NOTIFICATION_ID = 2
    }
}
