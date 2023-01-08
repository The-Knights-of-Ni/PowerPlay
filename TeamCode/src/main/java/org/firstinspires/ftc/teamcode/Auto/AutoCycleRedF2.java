package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.Subsystems.Vision.ConeColorPipeline.ConeColor;
import org.firstinspires.ftc.teamcode.Util.AllianceColor;
import org.firstinspires.ftc.teamcode.Util.Vector;

@Autonomous(name = "Auto Cycle Red F2")
public class AutoCycleRedF2 extends Auto {
    @Override
    public void runOpMode() throws InterruptedException {
        initAuto(AllianceColor.RED);
        waitForStart();
        timer.reset();
        ConeColor coneColor = robot.vision.detectConeColor();
        telemetry.addData("Cone Color", coneColor);
        robot.drive.moveVector(new Vector(0*mmPerInch, 48*mmPerInch));
        robot.drive.moveVector(new Vector(12*mmPerInch, 0*mmPerInch));
        // TODO: Score preloaded here
        robot.drive.moveVector(new Vector(-12*mmPerInch, 0));
        robot.drive.moveVector(new Vector(0, 0), -90);
        robot.drive.moveVector(new Vector(0*mmPerInch, -21.5*mmPerInch));
        // TODO: Pickup cone
        robot.drive.moveVector(new Vector(0*mmPerInch, 21.5*mmPerInch));
        robot.drive.moveVector(new Vector(0*mmPerInch, 0*mmPerInch), 45);
        // TODO: Score/Cycle
        // TODO: Park based on cone color

        sleep(2000);
    }
}
