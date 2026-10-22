package dev.khyber.ride

import android.app.Notification
import android.content.ComponentName
import android.content.Context
import android.provider.Settings
import android.service.notification.NotificationListenerService
import android.service.notification.StatusBarNotification
import dev.khyber.ride.proto.WalletNotification

/**
 * Records Google Wallet's notifications (e.g. after an OMNY tap) into the ride's `radio` stream while recording.
 * Every other app's notifications are ignored and never stored. Android only runs this once the rider grants
 * notification access in Settings.
 */
class WalletListener : NotificationListenerService() {
    override fun onNotificationPosted(sbn: StatusBarNotification) {
        if (sbn.packageName !in WALLET_PACKAGES) return
        val records = sink ?: return
        Health.guard("wallet notification") {
            val extras = sbn.notification.extras
            val n = WalletNotification.newBuilder()
                .setPackageName(sbn.packageName)
                .setTitle(extras.getCharSequence(Notification.EXTRA_TITLE)?.toString().orEmpty())
                .setText(extras.getCharSequence(Notification.EXTRA_TEXT)?.toString().orEmpty())
                .setPostTimeMs(sbn.postTime)
                .build()
            records.add { walletNotification = n }
        }
    }

    companion object {
        private val WALLET_PACKAGES = setOf("com.google.android.apps.walletnfcrel")

        /** Where to record, while recording; set by [RecorderService]. */
        @Volatile
        var sink: RecordStreamRecorder? = null

        fun allowed(context: Context): Boolean {
            val enabled = Settings.Secure.getString(context.contentResolver, "enabled_notification_listeners").orEmpty()
            return ComponentName(context, WalletListener::class.java).flattenToString() in enabled
        }
    }
}
