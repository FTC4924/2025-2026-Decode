package org.firstinspires.ftc.teamcode.autonomous.park

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Vector2d
import com.acmerobotics.roadrunner.ftc.runBlocking
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.teamcode.LocationShare
import org.firstinspires.ftc.teamcode.roadrunner.IHDrive


@Autonomous
class LeaveRedWall : OpMode() {
    lateinit var drive: IHDrive
    val beginPose = Pose2d(61.0, -15.0, -Math.PI / 2) // was x = 30.5, y = 66

    override fun init() {
        drive = IHDrive(hardwareMap, beginPose)
    }

    override fun start() {
        runBlocking(
            drive.actionBuilder(beginPose)
                .strafeTo(Vector2d(57.0, -39.0))
                .build()
        )
    }

    override fun loop() {
        TODO("Not yet implemented")
    }

    override fun stop() {
        LocationShare.robotLocation = drive.localizer.pose
    }
}