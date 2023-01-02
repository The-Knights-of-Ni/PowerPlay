package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Subsystems.Vision.ConeColorPipeline.ConeColor;
import org.firstinspires.ftc.teamcode.Util.AllianceColor;

@Autonomous(name = "Auto Park")
public class AutoPark extends Auto{
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
        ConeColor coneColor = robot.vision.detectConeColor();
        telemetry.addData("Cone Color", coneColor);
        switch (coneColor) {
            case GREEN:
                robot.drive.moveLeft(24*mmPerInch);
                robot.drive.moveForward(36*mmPerInch);
            case PINK:
                robot.drive.moveForward(36*mmPerInch);
            case ORANGE:
                robot.drive.moveRight(24*mmPerInch);
                robot.drive.moveForward(36*mmPerInch);
        }
    }
}
