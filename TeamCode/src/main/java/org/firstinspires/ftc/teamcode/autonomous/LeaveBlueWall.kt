package org.firstinspires.ftc.teamcode.autonomous.park

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Vector2d
import com.acmerobotics.roadrunner.ftc.runBlocking
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive


@Autonomous
class LeaveBlueWall : LinearOpMode() {
    override fun runOpMode() {
        val beginPose = Pose2d(61.0, 15.0, Math.PI / 2) // was x = 30.5, y = 66
        val drive = PinpointDrive(hardwareMap, beginPose)
        waitForStart()
        runBlocking(
            drive.actionBuilder(beginPose)
                .strafeTo(Vector2d(57.0, 39.0))
                .build()
        )
    }
}