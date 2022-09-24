package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Util.AllianceColor;

@Autonomous(name = "Auto Cycle Blue", group = "Concept")
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
        robot.drive.moveRight(24*mmPerInch);
        robot.drive.moveForward(48*mmPerInch);
        // TODO: Pick up Cone
        robot.drive.moveLeft(22*mmPerInch);
        // TODO: Insert Scoring code
        robot.drive.moveRight(22*mmPerInch);
    }
}
