package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Subsystems.Control.Control;
import org.firstinspires.ftc.teamcode.Subsystems.Vision.ConeColorPipeline.ConeColor;
import org.firstinspires.ftc.teamcode.Util.AllianceColor;
import org.firstinspires.ftc.teamcode.Util.Vector;

@Autonomous(name = "Auto Cycle Red F2")
public class AutoCycleRedF2 extends Auto {
    @Override
    public void runOpMode() throws InterruptedException {
        // ALWAYS START ROBOT 3 in. from edge of the tile that's farther from center of field
        // Initialize devices
        initAuto(AllianceColor.RED);
        waitForStart();
        timer.reset();

        // Scan for cone color, display on telemetry
        ConeColor coneColor = robot.vision.detectConeColor();
        telemetry.addData("Cone Color", coneColor.color);
        telemetry.update();
        robot.vision.stop();

        // Manual override to get the cone above floor
        robot.bar.setTargetPosition(200);
        robot.bar.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        // Move towards high goal & score pre-loaded
        robot.drive.moveVector(new Vector(0*mmPerInch, 48*mmPerInch));
        robot.control.deploy(Control.BarState.HIGH);
        robot.drive.moveVector(new Vector(10*mmPerInch, 0*mmPerInch));
        sleep(2000);
        robot.drive.moveVector(new Vector(0*mmPerInch, 5*mmPerInch));
        robot.control.toggleClaw(Control.ClawState.OPEN);
        sleep(500);

        // TODO Calibrate to cycle (will do after qualifiers)

        // Park based on cone color
        switch(coneColor) {
            case GREEN:
                robot.drive.moveVector(new Vector(-30*mmPerInch, 0));
                break;
            case PINK:
                robot.drive.moveVector(new Vector(-10*mmPerInch, 0));
                break;
            case ORANGE:
                robot.drive.moveVector(new Vector(14*mmPerInch, 0));
                break;
        }
        robot.control.retract();
        sleep(5000);
    }
}
