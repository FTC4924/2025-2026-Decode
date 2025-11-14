package org.firstinspires.ftc.teamcode.subsystems

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap


class Ramp(hardwareMap: HardwareMap) {

    /**
     * @param position the position of the scoringArm in that state, -1 means we don't currently
     * know the position of the scoringArm
     */
    enum class RampState(val position: Int) {
        Shoot(260), // Need to test and find correct values
        Collect(360), //
        Partner (0), //
        Manual(-1)
    }

    var rampState = RampState.Collect

    val ramp = hardwareMap.get(DcMotor::class.java, "ramp")

    private val power = 0.75

    var rampOffset = 0 //offset used to reset the arm positions mid-match
    var targetPosition = 0.0    //When program/class is initialized, assume start at 0

    init {
        ramp.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        ramp.targetPosition = 0
        ramp.mode = DcMotor.RunMode.RUN_TO_POSITION
        ramp.power = power
    }

    /**
     * Start: sets the arm state and position;
     * IsFinished: the arm has reached the state's position
     *
     * @param state the state (and associated position) to set the arm to
     */
    inner class SetState(private val state: RampState) : Action {
        private var initialized = false

        @Suppress("PARAMETER_NAME_CHANGED_ON_OVERRIDE")
        override fun run(packet: TelemetryPacket): Boolean {
            if (!initialized) {
                targetPosition = state.position.toDouble()
                ramp.targetPosition = targetPosition.toInt() - rampOffset
                rampState= state
                initialized = true
            }
            //scoringArm.currentPosition
            packet.put("Target Position", ramp.targetPosition)
            packet.put("Current Position", ramp.currentPosition)
            return ramp.isBusy
        }
    }

    /**
     * manually changes the position of the scoringArm (typically with a joystick)
     *
     * @param input the percent speed (-1 to 1) normalized by delta time (the time between each loop)
     */
    inner class Manual(private val input: Double) : Action {
        val maxSpeed = 600.0 //tics/second

        @Suppress("PARAMETER_NAME_CHANGED_ON_OVERRIDE")
        override fun run(packet: TelemetryPacket): Boolean {
            rampState = RampState.Manual
            targetPosition += input * maxSpeed
            ramp.targetPosition = targetPosition.toInt() - rampOffset
            packet.put("Target Position", ramp.targetPosition)
            packet.put("Current Position", ramp.currentPosition)
            return false
        }
    }

    /**
     * Only use in the collect position; used to reset the positions of the arm; should be called
     * alongside a collect action
     */
    fun resetRampPosition() {
        rampOffset = RampState.Collect.position - ramp.currentPosition

    }

    fun shoot(): Action = SetState(RampState.Shoot)
    fun collect(): Action = SetState(RampState.Collect)
    fun partner(): Action = SetState(RampState.Partner)
    fun manual(input: Double): Action = Manual(input)
}