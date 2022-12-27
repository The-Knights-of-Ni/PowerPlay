package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Util.AllianceColor;

@Autonomous(name = "Auto Cycle Blue A2")
public class AutoCycleBlueA2 extends Auto {
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
        telemetry.addData("cone color", robot.vision.detectConeColor());
        waitForStart();
        timer.reset();
        robot.drive.moveRight(-24*mmPerInch);
        robot.drive.moveForward(48*mmPerInch);
        robot.drive.turnByAngle(90);
        while (timer.seconds() < 20) {
            // TODO: Insert Intake code
            robot.drive.moveForward(22 * mmPerInch);
            // TODO: Insert Scoring code
            robot.drive.moveBackward(22 * mmPerInch);
        }
    }
}
