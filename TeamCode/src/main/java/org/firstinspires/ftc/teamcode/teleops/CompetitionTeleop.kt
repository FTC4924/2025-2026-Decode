package org.firstinspires.ftc.teamcode.teleops

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.Rotation2d
import com.acmerobotics.roadrunner.SequentialAction
import com.acmerobotics.roadrunner.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import org.firstinspires.ftc.teamcode.subsystems.Ramp
import org.firstinspires.ftc.teamcode.subsystems.Ramp.RampState
import org.firstinspires.ftc.teamcode.subsystems.Shooter
import org.firstinspires.ftc.teamcode.subsystems.Shooter.ShooterState
import org.firstinspires.ftc.teamcode.subsystems.Collection
import org.firstinspires.ftc.teamcode.subsystems.Collection.CollectionState

@TeleOp(name = "CompetitionTeleop")
class CompetitionTeleop : OpMode() {
    private lateinit var drive: PinpointDrive
    private lateinit var g1: PandaGamepad
    private lateinit var g2: PandaGamepad
    private lateinit var shooter: Shooter
    private lateinit var collection: Collection
    private lateinit var ramp: Ramp
    private var headingOffset: Double = 0.0
    private val dash: FtcDashboard = FtcDashboard.getInstance()
    private var runningActions: MutableList<Action> = ArrayList()
    private var lastTime: Double = 0.0

    override fun init() {
        drive = PinpointDrive(hardwareMap, Pose2d(0.0, 0.0, 0.0))
        g1 = PandaGamepad(gamepad1)
        g2 = PandaGamepad(gamepad2)
        shooter = Shooter(hardwareMap)
        ramp = Ramp(hardwareMap)
        collection = Collection(hardwareMap)
    }

    override fun start() {
        lastTime = runtime
    }

    fun getDeltaTime(): Double {
        val newTime = runtime
        val deltaTime: Double = newTime - lastTime
        lastTime = newTime
        return deltaTime
    }

    fun nearestCompass(currentHeading: Double, direction: String): Double {
        //Finds nearest 90 degree increment to current heading
        var headOut: Double = 0.0

        if ((currentHeading < 90) and (currentHeading <= 0)) headOut = 90.0
        else if ((currentHeading < 180) and (currentHeading <= 90)) headOut = 180.0
        else if ((currentHeading < -90) and (currentHeading <= -180)) headOut = -90.0
        else if ((currentHeading < 0) and (currentHeading <= -90)) headOut = 0.0

        if (direction == "right") return headOut
        else if (direction == "left") return headOut - 90
        else return 0.0
    }

    override fun loop() {
        //update delta time
        val deltaTime = getDeltaTime()
        telemetry.addData("Delta Time", deltaTime)

        //update gamepad values
        g1.update()
        g2.update()

        //run and update actions
        val packet = PandaTelemetryPacket(telemetry)

        val newActions: MutableList<Action> = ArrayList()
        for (action in runningActions) {
            action.preview(packet.fieldOverlay())
            telemetry.addLine(action.toString())
            if (action.run(packet)) {
                newActions.add(action)
            }
        }
        runningActions = newActions

        telemetry.addData("position", ramp.ramp.currentPosition)

        dash.sendTelemetryPacket(packet)

        //update drive Pose
        drive.updatePoseEstimate()

        /* driver 1 */
        val rawHeading = drive.getPinpoint().heading
        val heading: Rotation2d = Rotation2d.fromDouble(rawHeading - headingOffset)
        val slowSpeed = 0.7 //Normally go about 70% of our fastest speed

        val input = Vector2d(
            g1.leftStickY.component.toDouble(),
            -g1.leftStickX.component.toDouble()
        )
        if (g1.y.isActive()) {  //When Boosting
            drive.setDrivePowers(
                PoseVelocity2d(
                    heading.inverse().times(input),
                    ((gamepad1.left_trigger - gamepad1.right_trigger) * 1 / 2).toDouble()
                )
            )
        } else {
            drive.setDrivePowers(
                PoseVelocity2d(
                    heading.inverse().times(input * slowSpeed),    //Coach Ethan added slow 1/19
                    ((gamepad1.left_trigger - gamepad1.right_trigger) * 1 / 2 * slowSpeed).toDouble()
                )
            )
        }


        if (g1.b.justPressed()) headingOffset = rawHeading

        /* driver 2 */

        if (g2.a.justPressed()) {
            runningActions.add(shooter.shooting())
        }
        else if (g2.a.justReleased()){
            runningActions.add(shooter.idle())
        }
        if (g2.dpadDown.justPressed()) {
            runningActions.add(ramp.partner())
        }


        if (g2.dpadRight.justPressed()) {
            runningActions.add(ramp.collect())
        }
        if (g2.dpadUp.justPressed()) {
            runningActions.add(ramp.shoot())
        }
        if (g2.dpadLeft.justPressed()) {
            ramp.resetRampPosition()
        }

        if (g2.leftStickY.isActive()) runningActions.add(ramp.manual(g2.leftStickY.component * deltaTime))

        if (g2.y.justPressed()) {
            if (collection.collectionState == CollectionState.Stopped) {
                runningActions.add(collection.collecting())
            } else if (collection.collectionState == CollectionState.Backward){
                runningActions.add(collection.collecting())
            } else {
                runningActions.add(collection.stopped())
            }
        }
        if (g2.x.justPressed()) {
            if (collection.collectionState == CollectionState.Stopped) {
                runningActions.add(collection.spitting())
            } else if (collection.collectionState == CollectionState.Forward)
                runningActions.add(collection.spitting())
            else {
                runningActions.add(collection.stopped())

            }
        }
    }

}