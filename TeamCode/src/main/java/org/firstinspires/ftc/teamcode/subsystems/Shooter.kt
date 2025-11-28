package org.firstinspires.ftc.teamcode.subsystems

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap


class Shooter(hardwareMap: HardwareMap) {

    /**
     * @param position the position of the scoringArm in that state, -1 means we don't currently
     * know the position of the scoringArm
     */
    enum class ShooterState(val power: Double) {
        Forward(1.0),
        Stopped(0.0),
        Idle(0.0)      //Need to test and update!!!

    }

    var shooterState = ShooterState.Stopped

    private val shooter = hardwareMap.get(CRServo::class.java, "shooter")


    var armPower = 0.0  //W hen program/class is initialized, assume start at 0

    init {
        shooter.direction = DcMotorSimple.Direction.REVERSE
    }


    inner class SetState(private val state: ShooterState) : Action {
        private var initialized = false

        @Suppress("PARAMETER_NAME_CHANGED_ON_OVERRIDE")
        override fun run(packet: TelemetryPacket): Boolean {
            if (!initialized) {
                armPower = state.power.toDouble()
                shooter.power = armPower.toDouble()
                shooterState = state
                initialized = true
            }
            return true
        }
    }


    fun shoot(): Action = SetState(ShooterState.Forward)
    fun stop(): Action = SetState(ShooterState.Stopped)
    fun idle(): Action = SetState (ShooterState.Idle)
}