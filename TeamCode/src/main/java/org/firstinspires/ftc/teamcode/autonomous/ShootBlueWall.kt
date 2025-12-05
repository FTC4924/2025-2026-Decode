package org.firstinspires.ftc.teamcode.autonomous
import org.firstinspires.ftc.teamcode.subsystems.Ramp
import org.firstinspires.ftc.teamcode.subsystems.Shooter
import org.firstinspires.ftc.teamcode.subsystems.Collection
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Vector2d
import com.acmerobotics.roadrunner.ftc.runBlocking
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.teamcode.roadrunner.IHDrive



@Autonomous
class ShootBlueWall : OpMode() {

    val beginPose = Pose2d(61.0, -15.0, -Math.PI / 2) // was x = 30.5, y = 66
    lateinit var shooter: Shooter
    lateinit var drive: IHDrive
    lateinit var ramp: Ramp
    lateinit var collection: Collection

     override fun init() {
         shooter = Shooter(hardwareMap)
         drive = IHDrive(hardwareMap, beginPose)
         ramp = Ramp(hardwareMap)
         collection = Collection(hardwareMap)
        }
    override fun start() {

        runBlocking(
            drive.actionBuilder(beginPose)
                .splineToLinearHeading(Pose2d(Vector2d(0.0, 0.0), 5*Math.PI/4), Math.PI)
                .build()
        )
        runBlocking(ramp.collect())
        runBlocking(collection.collectIn())
        runBlocking(shooter.shoot())
        runBlocking(
            drive.actionBuilder(drive.getPose())
                .waitSeconds(3.0)
                .build()
        )
        runBlocking(ramp.shoot())

    }

    override fun loop() {
    }
}
