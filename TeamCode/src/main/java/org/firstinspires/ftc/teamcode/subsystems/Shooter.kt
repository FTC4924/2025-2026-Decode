package org.firstinspires.ftc.teamcode.subsystems

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap


class Shooter(hardwareMap: HardwareMap) {

    val small: Double = 0.05
    val big: Double = 0.1
    var powerAdjustment = 0.0

    /**
     * @param position the position of the scoringArm in that state, -1 means we don't currently
     * know the position of the scoringArm
     */
    enum class ShooterState(val power: Double) {
        Forward(0.9),
        Stopped(0.0),
        Idle(0.0)      //Need to test and update!!!

    }

    var shooterState = ShooterState.Stopped

    val shooter = hardwareMap.get(CRServo::class.java, "shooter")

    init {
        shooter.direction = DcMotorSimple.Direction.REVERSE
        shooter.power = ShooterState.Stopped.power
    }


    inner class SetState(private val state: ShooterState) : Action {

        @Suppress("PARAMETER_NAME_CHANGED_ON_OVERRIDE")
        override fun run(packet: TelemetryPacket): Boolean {
            shooterState = state
            shooter.power = shooterState.power + powerAdjustment
            return false
        }
    }


    inner class AdjustPower(private val powerChange: Double)  : Action {

        @Suppress("PARAMETER_NAME_CHANGED_ON_OVERRIDE")
        override fun run(packet: TelemetryPacket): Boolean {
            powerAdjustment += powerChange
            shooter.power = shooterState.power + powerAdjustment
            return false
        }

    }

    fun shoot(): Action = SetState(ShooterState.Forward)
    fun stop(): Action = SetState(ShooterState.Stopped)
    fun idle(): Action = SetState (ShooterState.Idle)
    fun adjustPower(powerChange: Double): Action = AdjustPower(powerChange)
}