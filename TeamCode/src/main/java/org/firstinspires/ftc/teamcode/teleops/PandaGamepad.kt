@file:Suppress("MemberVisibilityCanBePrivate", "unused")

package org.firstinspires.ftc.teamcode.teleops

import androidx.annotation.FloatRange
import com.qualcomm.robotcore.hardware.Gamepad
import org.firstinspires.ftc.teamcode.teleops.PandaGamepad.ControllerElementState.*
import kotlin.math.abs

class PandaGamepad(val gamepad: Gamepad) {
    val a = ButtonComponent()
    val b = ButtonComponent()
    val x = ButtonComponent()
    val y = ButtonComponent()
    val dpadUp = ButtonComponent()
    val dpadRight = ButtonComponent()
    val dpadDown = ButtonComponent()
    val dpadLeft = ButtonComponent()
    val leftBumper = ButtonComponent()
    val rightBumper = ButtonComponent()
    val rightStickButton = ButtonComponent()
    val leftStickButton = ButtonComponent()
    val leftStickX = AnalogComponent(0.05)
    val leftStickY = AnalogComponent(0.05)
    val rightStickX = AnalogComponent(0.05)
    val rightStickY = AnalogComponent(0.05)
    val rightTrigger = AnalogComponent(0.05)
    val leftTrigger = AnalogComponent(0.05)

    fun update() {
        a.update(gamepad.a)
        b.update(gamepad.b)
        x.update(gamepad.x)
        y.update(gamepad.y)
        dpadUp.update(gamepad.dpad_up)
        dpadRight.update(gamepad.dpad_right)
        dpadDown.update(gamepad.dpad_down)
        dpadLeft.update(gamepad.dpad_left)
        leftBumper.update(gamepad.left_bumper)
        rightBumper.update(gamepad.right_bumper)
        rightStickButton.update(gamepad.right_stick_button)
        leftStickButton.update(gamepad.left_stick_button)
        leftStickX.update(gamepad.left_stick_x.toDouble())
        leftStickY.update((gamepad.left_stick_y * -1).toDouble())
        rightStickX.update(gamepad.right_stick_x.toDouble())
        rightStickY.update((gamepad.right_stick_y * -1).toDouble())
        rightTrigger.update(gamepad.right_trigger.toDouble())
        leftTrigger.update(gamepad.left_trigger.toDouble())
    }

    enum class ControllerElementState {
        Idle,
        Pressed,
        Held,
        Released
    }

    open class GamepadComponent {
        private var state = Idle

        open fun update(active: Boolean) {
            when (state) {
                Idle -> if (active) state = Pressed
                Pressed -> state = if (active) Held else Released
                Held -> if (!active) state = Released
                Released -> state = if (!active) Idle else Pressed
            }
        }

        fun justActive() = state == Pressed
        fun isActive() = state == Held
        fun justInactive() = state == Released
        fun isInactive() = state == Idle
    }

    class AnalogComponent(@FloatRange(from = 0.0, to = 1.0) val analogThreshold: Double) : GamepadComponent() {

        var component: Double = 0.0
            private set

        fun update(component: Double) {
            val active = abs(component) >= analogThreshold
            update(active)
            if (!isInactive()) this.component = component
        }
    }

    class ButtonComponent : GamepadComponent() {
        @Suppress("PARAMETER_NAME_CHANGED_ON_OVERRIDE", "RedundantOverride")
        override fun update(isDown: Boolean) = super.update(isDown)

        fun justPressed() = justActive()
        fun isHeld() = isActive()
        fun justReleased() = justInactive()
        fun isIdle() = isInactive()
    }
}