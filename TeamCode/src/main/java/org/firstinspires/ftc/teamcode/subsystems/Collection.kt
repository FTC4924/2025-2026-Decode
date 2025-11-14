package org.firstinspires.ftc.teamcode.subsystems

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo


class Collection(hardwareMap: HardwareMap) {

    /**
     * @param position the position of the scoringArm in that state, -1 means we don't currently
     * know the position of the scoringArm
     */
    enum class CollectionState(val power: Double) {
        Forward(1.0),
        Backward(-0.75),
        Stopped(0.0)

    }

    var collectionState = CollectionState.Stopped

    private val collection = hardwareMap.get(CRServo::class.java, "collection")


    var collectionPower = 0.0  //W hen program/class is initialized, assume start at 0

    init {
        collection.direction = DcMotorSimple.Direction.REVERSE
    }


    inner class SetState(private val state: CollectionState) : Action {
        private var initialized = false

        @Suppress("PARAMETER_NAME_CHANGED_ON_OVERRIDE")
        override fun run(packet: TelemetryPacket): Boolean {
            if (!initialized) {
                collectionPower = state.power.toDouble()
                collection.power = collectionPower.toDouble()
                collectionState = state
                initialized = true
            }
            return true
        }
    }


    fun collecting(): Action = SetState(CollectionState.Forward)
    fun spitting(): Action = SetState (CollectionState.Backward)
    fun stopped(): Action = SetState(CollectionState.Stopped)

}