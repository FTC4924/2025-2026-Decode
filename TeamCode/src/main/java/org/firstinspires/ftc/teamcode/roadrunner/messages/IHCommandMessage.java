package org.firstinspires.ftc.teamcode.roadrunner.messages;

public final class IHCommandMessage {
    public long timestamp;
    public double voltage;
    public double leftPower;
    public double rightPower;
    public double middlePower;

    public IHCommandMessage(double voltage, double leftPower, double rightPower, double rightBackPower) {
        this.timestamp = System.nanoTime();
        this.voltage = voltage;
        this.leftPower = leftPower;
        this.rightPower = rightPower;
        this.middlePower = rightBackPower;
    }
}
