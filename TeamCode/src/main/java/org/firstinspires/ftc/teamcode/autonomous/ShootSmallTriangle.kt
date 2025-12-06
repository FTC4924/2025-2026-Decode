package org.firstinspires.ftc.teamcode.autonomous
import org.firstinspires.ftc.teamcode.subsystems.Ramp
import org.firstinspires.ftc.teamcode.subsystems.Shooter
import org.firstinspires.ftc.teamcode.subsystems.Collection
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Vector2d
import com.acmerobotics.roadrunner.ftc.runBlocking
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.teamcode.roadrunner.IHDrive



@Autonomous
class ShootSmallTriangle : OpMode() {

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


        runBlocking(ramp.collect())
        runBlocking(collection.collectIn())
        runBlocking(shooter.shoot())
        runBlocking(shooter.adjustPower(.1))
        runBlocking(
            drive.actionBuilder(drive.getPose())
                .waitSeconds(3.0)
                .build()
        )
        runBlocking(ramp.shoot())
        runBlocking(
            drive.actionBuilder(drive.getPose())
                .waitSeconds(12.0)
                .build()
        )
        runBlocking(shooter.stop())
        runBlocking(ramp.toZero())
        runBlocking(
            drive.actionBuilder(beginPose)
                .strafeTo(Vector2d(57.0, 39.0))
                .build())

    }

    override fun loop() {
    }
}
