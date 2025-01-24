package org.firstinspires.ftc.teamcode.autonomous.park;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive;

@Autonomous
@Disabled
public class RedAutoRight extends LinearOpMode {

    public void runOpMode() {
        Pose2d beginPose = new Pose2d(17.5, -66, Math.PI / 2);
        PinpointDrive drive = new PinpointDrive(hardwareMap, beginPose);
        waitForStart();
        Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        .strafeTo(new Vector2d(60.5, -64))
                        .build());
    }
}