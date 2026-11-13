package io.github.samimreza.virtualduck

import android.app.AlarmManager
import android.app.Notification
import android.app.NotificationChannel
import android.app.NotificationManager
import android.app.PendingIntent
import android.content.BroadcastReceiver
import android.content.Context
import android.content.Intent
import android.os.SystemClock
import io.github.samimreza.virtualduck.home.HomeActivity
import io.github.samimreza.virtualduck.overlay.PetService
import io.github.samimreza.virtualduck.pet.Mood
import java.time.LocalDate
import java.time.LocalTime

/**
 * A nudge when the duck is put away and wants something — the loop that makes a pet a habit, kept
 * gentle so it never becomes a nag: checked every few hours, only between 9:00 and 21:00, at most
 * [PER_DAY] a day, only for a reason (hungry, lonely, bored, a streak about to break), and never
 * while the duck is already out on the screen.
 *
 * An inexact repeating alarm, which the system batches with others — no exact-alarm permission, no
 * wakeups of its own.
 */
object Reminders {
    private const val PER_DAY = 2
    private const val CHANNEL = "reminders"
    private const val NOTIFICATION = 2

    fun schedule(context: Context) {
        val alarms = context.getSystemService(AlarmManager::class.java) ?: return
        val every = AlarmManager.INTERVAL_HOUR * 3
        alarms.setInexactRepeating(
            AlarmManager.ELAPSED_REALTIME,
            SystemClock.elapsedRealtime() + every,
            every,
            PendingIntent.getBroadcast(
                context, 0, Intent(context, ReminderReceiver::class.java),
                PendingIntent.FLAG_IMMUTABLE or PendingIntent.FLAG_UPDATE_CURRENT,
            ),
        )
    }

    /** What, if anything, is worth a nudge right now; and posts it. */
    fun check(context: Context) {
        if (PetService.running) return
        val hour = LocalTime.now().hour
        if (hour < 9 || hour >= 21) return
        val pet = PetHub.load(context) ?: return
        pet.catchUp(System.currentTimeMillis())
        val today = LocalDate.now().toEpochDay()
        if (pet.remindedDay != today) {
            pet.remindedDay = today
            pet.remindersToday = 0
        }
        if (pet.remindersToday >= PER_DAY) return
        val text = when {
            pet.mood == Mood.HUNGRY -> context.getString(R.string.remind_hungry, pet.name)
            pet.mood == Mood.LONELY -> context.getString(R.string.remind_lonely, pet.name)
            pet.mood == Mood.BORED -> context.getString(R.string.remind_bored, pet.name)
            pet.streak >= 2 && pet.lastCareDay < today && hour >= 17 -> context.getString(R.string.remind_streak, pet.streak, pet.name)
            else -> return
        }
        pet.remindersToday++
        PetHub.write(context, pet)
        post(context, text)
    }

    private fun post(context: Context, text: String) {
        val manager = context.getSystemService(NotificationManager::class.java) ?: return
        manager.createNotificationChannel(
            NotificationChannel(CHANNEL, context.getString(R.string.reminders_channel), NotificationManager.IMPORTANCE_DEFAULT),
        )
        val open = PendingIntent.getActivity(
            context, 3, Intent(context, HomeActivity::class.java),
            PendingIntent.FLAG_IMMUTABLE or PendingIntent.FLAG_UPDATE_CURRENT,
        )
        manager.notify(
            NOTIFICATION,
            Notification.Builder(context, CHANNEL)
                .setSmallIcon(R.drawable.ic_stat_duck)
                .setContentTitle(text)
                .setContentText(context.getString(R.string.remind_tap))
                .setContentIntent(open)
                .setAutoCancel(true)
                .build(),
        )
    }
}

class ReminderReceiver : BroadcastReceiver() {
    override fun onReceive(context: Context, intent: Intent) {
        runCatching { Reminders.check(context) }
    }
}
