package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.Subsystems.Vision.ConeColorPipeline.ConeColor;
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
        waitForStart();
        timer.reset();
        ConeColor coneColor = robot.vision.detectConeColor();
        telemetry.addData("Cone Color", coneColor);
        robot.drive.moveVector(new Vector(0 * mmPerInch, 48 * mmPerInch), 11.3);
        while (timer.seconds() < 20) {
            // TODO: Insert Scoring code
            robot.drive.moveVector(new Vector(-12 * mmPerInch, -24 * mmPerInch), 45);
            // TODO: Insert Intake code
            robot.drive.moveVector(new Vector(12 * mmPerInch, 24 * mmPerInch), -45);
        }
        switch (coneColor) {
            case PINK:
                robot.drive.moveVector(new Vector(0, 24 * mmPerInch));
            case ORANGE:
                robot.drive.moveVector(new Vector(0, 48 * mmPerInch));
        }
    }
}
