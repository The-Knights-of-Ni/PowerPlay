package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Subsystems.DetectMarkerPipeline;
import org.firstinspires.ftc.teamcode.Util.AllianceColor;

import java.io.IOException;

@Autonomous(name = "Vision Test", group = "Concept")
public class VisionTest extends LinearOpMode {
    private Robot robot;
    private ElapsedTime timer;

    private void initOpMode() throws IOException {
        telemetry.addData("Init Robot", "");
        telemetry.update();
        timer = new ElapsedTime();

        this.robot =  new Robot(this, hardwareMap, telemetry, timer, AllianceColor.BLUE, gamepad1, gamepad2,true);

        robot.telemetryBroadcast("Wait for start", "");

    }

    @Override
    public void runOpMode() throws InterruptedException{
        try {
            this.initOpMode();
        } catch (IOException e) {
            e.printStackTrace();
        }

        DetectMarkerPipeline.MarkerLocation location;

        waitForStart();

        while (opModeIsActive()) {
            location = robot.vision.detectMarkerRun();
            switch (location) {
                case LEFT:
                    telemetry.addData("Location", "Left");
                    break;
                case MIDDLE:
                    telemetry.addData("Location", "Middle");
                    break;
                case RIGHT:
                    telemetry.addData("Location", "Right");
                    break;
                case NOT_FOUND:
                    telemetry.addData("Location", "Not Found");
                    break;
            }
            telemetry.update();
        }
    }
}
