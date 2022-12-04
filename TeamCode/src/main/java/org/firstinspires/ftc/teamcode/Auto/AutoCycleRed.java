package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Util.AllianceColor;
import org.firstinspires.ftc.teamcode.Util.Vector;

@Autonomous(name = "Auto Cycle Red")
public class AutoCycleRed extends Auto {
    /**
     * Override of runOpMode()
     *
     * <p>Please do not swallow the InterruptedException, as it is used in cases where the op mode
     * needs to be terminated early.</p>
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
     */
    @Override
    public void runOpMode() throws InterruptedException {
        initAuto(AllianceColor.RED);
        waitForStart();
        timer.reset();
        telemetry.addData("cone color", robot.visionEnabled ? robot.vision.detectConeColor(): "");
        robot.drive.moveVector(new Vector(12*mmPerInch, 12*mmPerInch));
//        while (timer.seconds() < 20) {
//            // TODO: Insert Intake code
//            robot.drive.moveForward(22 * mmPerInch);
//            // TODO: Insert Scoring code
//            robot.drive.moveBackward(22 * mmPerInch);
        }
    }
