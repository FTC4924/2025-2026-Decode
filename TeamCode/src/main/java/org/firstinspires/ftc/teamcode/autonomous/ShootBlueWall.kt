package org.firstinspires.ftc.teamcode.autonomous
import org.firstinspires.ftc.teamcode.subsystems.Ramp
import org.firstinspires.ftc.teamcode.subsystems.Shooter
import org.firstinspires.ftc.teamcode.subsystems.Collection
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.SequentialAction
import com.acmerobotics.roadrunner.Vector2d
import com.acmerobotics.roadrunner.ftc.runBlocking
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.teamcode.LocationShare
import org.firstinspires.ftc.teamcode.roadrunner.IHDrive
import org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive
import org.firstinspires.ftc.teamcode.teleops.PandaTelemetryPacket


@Autonomous
class ShootBlueWall : OpMode() {

    val beginPose = Pose2d(61.0, -15.0, Math.PI) // was x = 30.5, y = 66
    lateinit var shooter: Shooter
    lateinit var drive: PinpointDrive
    lateinit var ramp: Ramp
    lateinit var collection: Collection
    var finished = false
    lateinit var autoAction: Action
     override fun init() {
         shooter = Shooter(hardwareMap)
         drive = PinpointDrive(hardwareMap, beginPose)
         ramp = Ramp(hardwareMap)
         collection = Collection(hardwareMap)

         autoAction = SequentialAction(
             drive.actionBuilder(beginPose)
                 .splineToLinearHeading(Pose2d(Vector2d(-12.0, -12.0), -(Math.PI / 4)), Math.PI)
                 .build(),
             ramp.collect(),
             drive.actionBuilder(drive.getPose())
                 .waitSeconds(1.0)
                 .build(),
             shooter.shoot(),
             drive.actionBuilder(drive.getPose())
                 .waitSeconds(3.0)
                 .build(),
             shooter.feed(),
             drive.actionBuilder(drive.getPose())
                 .waitSeconds(1.0)
                 .build(),
             ramp.shoot()
         )
     }
    override fun loop() {
        val packet = PandaTelemetryPacket(telemetry)
        if (!finished) {
            if (!autoAction.run(packet)) finished = true
        }
        //update drive Pose
        drive.updatePoseEstimate()

        telemetry.addData("x", drive.pose.position.x)
        telemetry.addData("y", drive.pose.position.y)
        telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()))
    }

    override fun stop() {
        LocationShare.robotLocation = drive.getPose()
    }


}
