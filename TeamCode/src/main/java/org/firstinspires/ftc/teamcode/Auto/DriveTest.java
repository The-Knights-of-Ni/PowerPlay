package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.firstinspires.ftc.teamcode.Util.AllianceColor;
import org.firstinspires.ftc.teamcode.Util.Vector;

@Autonomous(name = "Drive Test")
public class DriveTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        DcMotorEx fl = (DcMotorEx) hardwareMap.get("fl");
        DcMotorEx fr = (DcMotorEx) hardwareMap.get("fr");
        DcMotorEx rl = (DcMotorEx) hardwareMap.get("rl");
        DcMotorEx rr = (DcMotorEx) hardwareMap.get("rr");
        fl.setPower(1);
        fr.setPower(1);
        rl.setPower(1);
        rr.setPower(1);
    }
}
