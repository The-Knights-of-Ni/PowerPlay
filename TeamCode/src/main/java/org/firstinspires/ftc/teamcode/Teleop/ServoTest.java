package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Subsystems.Control;
import org.firstinspires.ftc.teamcode.Util.AllianceColor;

import java.io.IOException;

@TeleOp(name = "Servo Test", group= "Concept")
public class ServoTest extends LinearOpMode {
    private Robot robot;
    public ElapsedTime timer;
    double absIncrementStep = 0.005;

    private void initOpMode() throws IOException {
        telemetry.addData("Init Robot", "");
        telemetry.update();
        timer = new ElapsedTime();

        this.robot =  new Robot(hardwareMap, telemetry, timer, AllianceColor.BLUE, gamepad1, gamepad2,false);

        robot.telemetryBroadcast("wait for start", "");
    }

    @Override
    public void runOpMode() throws InterruptedException {
        try {
            initOpMode();
        } catch (IOException e) {
            e.printStackTrace();
        }
        waitForStart();
        while(opModeIsActive()) {
            robot.getGamePadInputs();

//            robot.markerSlide.setPosition(0.425);
//            robot.markerHook.setPosition(0.375);
        }
    }

}
