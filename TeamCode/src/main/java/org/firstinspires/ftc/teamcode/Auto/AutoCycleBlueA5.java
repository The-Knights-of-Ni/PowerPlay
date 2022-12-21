package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Subsystems.Vision.ConeColorPipeline;
import org.firstinspires.ftc.teamcode.Util.AllianceColor;
import org.firstinspires.ftc.teamcode.Util.Vector;

@Autonomous(name = "Auto Cycle Blue A5")
public class AutoCycleBlueA5 extends Auto {
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

        initAuto(AllianceColor.BLUE);
        ConeColorPipeline.ConeColor coneColor = robot.vision.detectConeColor();
        waitForStart();
        timer.reset();
        robot.drive.moveVector(new Vector(24*mmPerInch, 0), -90);
        robot.drive.moveRight(48*mmPerInch);
        robot.drive.moveForward(22 * mmPerInch);
        while (timer.seconds() < 20) {
            // TODO: Insert Scoring code
            robot.drive.moveBackward(22 * mmPerInch);
            // TODO: Insert Intake code
            robot.drive.moveForward(22 * mmPerInch);
        }
        if (coneColor == ConeColorPipeline.ConeColor.GREEN) {
            robot.drive.moveForward(22 * mmPerInch);
        }
        else if (coneColor == ConeColorPipeline.ConeColor.BLUE) {
            robot.drive.moveForward(12 * mmPerInch);
        }
    }
}
