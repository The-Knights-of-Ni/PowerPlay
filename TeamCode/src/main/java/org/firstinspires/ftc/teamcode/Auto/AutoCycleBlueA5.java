package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Subsystems.Vision.ConeColorPipeline.ConeColor;
import org.firstinspires.ftc.teamcode.Util.AllianceColor;
import org.firstinspires.ftc.teamcode.Util.Vector;

@Autonomous(name = "Auto Cycle Blue A5")
public class AutoCycleBlueA5 extends Auto {
    @Override
    public void runOpMode() throws InterruptedException {
        initAuto(AllianceColor.BLUE);
        waitForStart();
        timer.reset();
        ConeColor coneColor = robot.vision.detectConeColor();
        telemetry.addData("Cone Color", coneColor);
        robot.drive.moveVector(new Vector(12*mmPerInch, 12*mmPerInch));
        robot.drive.moveVector(new Vector(0*mmPerInch, 96*mmPerInch));
        while (timer.seconds() < 20) {
            robot.drive.moveVector(new Vector(-12*mmPerInch, -24*mmPerInch), 0);
            // TODO: Insert Intake code
            robot.drive.moveVector(new Vector(12*mmPerInch, 24*mmPerInch), 0);
        }
        switch (coneColor) {
            case PINK:
                robot.drive.moveVector(new Vector(0, 24*mmPerInch));
            case ORANGE:
                robot.drive.moveVector(new Vector(0, 48*mmPerInch));
        }
    }
}
