package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Subsystems.Vision.ConeColorPipeline.ConeColor;
import org.firstinspires.ftc.teamcode.Util.AllianceColor;
import org.firstinspires.ftc.teamcode.Util.Vector;

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
        telemetry.addData("Cone Color", coneColor.color);
        telemetry.update();
        switch (coneColor) {
            case GREEN:
                robot.drive.moveVector(new Vector(-20*mmPerInch, 0));
                robot.drive.moveVector(new Vector(0, 24*mmPerInch));
                break;
            case PINK:
                robot.drive.moveVector(new Vector(0, 24*mmPerInch));
                break;
            case ORANGE:
                robot.drive.moveVector(new Vector(20*mmPerInch, 0));
                robot.drive.moveVector(new Vector(0, 24*mmPerInch));
                break;
        }
    }
}
