package org.firstinspires.ftc.teamcode.roadrunner.tuning;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.IHDrive;

public final class SplineTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(61.0, -15.0, Math.PI);
        if (TuningOpModes.DRIVE_CLASS.equals(IHDrive.class)) {
            IHDrive drive = new IHDrive(hardwareMap, beginPose);

            waitForStart();

            Actions.runBlocking(
                    drive.actionBuilder(beginPose)
                            .setTangent(-3 * Math.PI/4)
                            .splineToSplineHeading(new Pose2d(new Vector2d(-12.0, -12.0), -(Math.PI / 4)), 3 * Math.PI/4)
                            .build()
            );
        } else {
            throw new RuntimeException();
        }
    }
}
