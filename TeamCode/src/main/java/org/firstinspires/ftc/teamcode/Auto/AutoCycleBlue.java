package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Util.AllianceColor;
import org.firstinspires.ftc.teamcode.Util.Vector;

@Autonomous(name = "Auto Cycle Blue")
public class AutoCycleBlue extends Auto {
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
        waitForStart();
        timer.reset();
        robot.drive.moveVector(new Vector(0, -48*mmPerInch));
        robot.drive.moveVector(new Vector(24*mmPerInch, 48*mmPerInch));
        robot.drive.moveVector(new Vector(-24*mmPerInch, 0));
        robot.drive.turnByAngle(90);
//        while (timer.seconds() < 20) {
//            // TODO: Insert Intake code
//            robot.drive.moveForward(22 * mmPerInch);
//            // TODO: Insert Scoring code
//            robot.drive.moveBackward(22 * mmPerInch);
//        }
    }
}
