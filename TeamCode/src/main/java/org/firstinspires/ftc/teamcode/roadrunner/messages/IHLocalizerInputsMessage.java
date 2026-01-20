package org.firstinspires.ftc.teamcode.roadrunner.messages;

import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public final class IHLocalizerInputsMessage {
    public long timestamp;
    public PositionVelocityPair left;
    public PositionVelocityPair right;
    public PositionVelocityPair middle;
    public double yaw;
    public double pitch;
    public double roll;

    public IHLocalizerInputsMessage(PositionVelocityPair left, PositionVelocityPair right, PositionVelocityPair middle, YawPitchRollAngles angles) {
        this.timestamp = System.nanoTime();
        this.left = left;
        this.right = right;
        this.middle = middle;
        {
            this.yaw = angles.getYaw(AngleUnit.RADIANS);
            this.pitch = angles.getPitch(AngleUnit.RADIANS);
            this.roll = angles.getRoll(AngleUnit.RADIANS);
        }
    }
}
