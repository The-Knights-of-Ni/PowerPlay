package org.firstinspires.ftc.teamcode.PurePursuit;

public class MotorSpeed {
    public final double duration; // in milliseconds
    public final double fl;
    public final double fr;
    public final double bl;
    public final double br;
    public MotorSpeed(double frontLeft, double frontRight, double backLeft, double backRight) {
        duration = 1;
        fl = frontLeft;
        fr = frontRight;
        bl = backLeft;
        br = backRight;
    }

    public MotorSpeed(double duration, double frontLeft, double frontRight, double backLeft, double backRight) {
        this.duration = duration;
        fl = frontLeft;
        fr = frontRight;
        bl = backLeft;
        br = backRight;
    }

}
