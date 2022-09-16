package org.firstinspires.ftc.teamcode.Auto;

import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Util.AllianceColor;

import java.io.IOException;

@Autonomous(name = "CR Servo Test", group = "Concept")
public class CRServoTest extends LinearOpMode {
    private Robot robot;
    public ElapsedTime timer;

    private void initOpMode() throws IOException {
        telemetry.addData("Init Robot", "");
        telemetry.update();
        timer = new ElapsedTime();

        this.robot =  new Robot(this, hardwareMap, telemetry, timer, AllianceColor.BLUE, gamepad1, gamepad2,true);

        telemetry.addData("Wait for start", "");
        telemetry.update();
    }

    @Override
    public void runOpMode() throws InterruptedException{
//        try {
//            initOpMode();
//        } catch (IOException e) {
//            e.printStackTrace();
//        }

        CRServo hi = new CRServo(hardwareMap, "duckWheel");
        telemetry.addLine("done!");
        telemetry.update();

        waitForStart();
        hi.set(1.0);
        //hi.set(0);
        //robot.duckWheel.setPosition(0.5);
    }
}