package dev.khyber.ride

import android.view.HapticFeedbackConstants
import androidx.compose.foundation.layout.RowScope
import androidx.compose.material3.Button
import androidx.compose.runtime.Composable
import androidx.compose.ui.Modifier
import androidx.compose.ui.platform.LocalView

/** A `Button` that also buzzes when tapped, so a tap can be felt without looking at the screen. */
@Composable
fun HapticButton(
    onClick: () -> Unit,
    modifier: Modifier = Modifier,
    enabled: Boolean = true,
    content: @Composable RowScope.() -> Unit,
) {
    val view = LocalView.current
    Button(
        onClick = {
            view.performHapticFeedback(HapticFeedbackConstants.CONFIRM)
            onClick()
        },
        modifier = modifier,
        enabled = enabled,
        content = content,
    )
}
