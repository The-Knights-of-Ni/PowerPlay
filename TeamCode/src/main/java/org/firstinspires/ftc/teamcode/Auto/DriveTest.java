package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.firstinspires.ftc.teamcode.Util.AllianceColor;

@Autonomous(name = "Drive Test", group = "Auto")
public class DriveTest extends Auto {
    @Override
    public void runOpMode() throws InterruptedException {
        Vector2D move_vector = new Vector2D(20, 20);
        initAuto(AllianceColor.BLUE);
        waitForStart();
        robot.drive.moveVector(move_vector, 1.0);
    }
}
