package org.firstinspires.ftc.teamcode.subsystems

import com.qualcomm.hardware.limelightvision.LLResult
import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.hardware.HardwareMap

class Camera (hardwareMap: HardwareMap) {
    private val limelight = hardwareMap.get(Limelight3A::class.java, "limelight")
    init {
        limelight.pipelineSwitch(0)
        limelight.start()
    }
    var LLresults: LLResult = limelight.getLatestResult()

    fun updateLL(): Boolean {
        val result: LLResult = limelight.getLatestResult()
        if (result != null && result.isValid()) {
            LLresults = result
            return true
        } else {
            return false
        }
    }
}