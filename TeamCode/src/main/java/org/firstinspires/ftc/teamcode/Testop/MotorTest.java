package org.firstinspires.ftc.teamcode.Testop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Subsystems.Control.Control;
import org.firstinspires.ftc.teamcode.Util.AllianceColor;

import java.util.HashMap;

@TeleOp(name = "Motor Position Test")
public class MotorTest extends LinearOpMode {
    private Robot robot;
    private double robotAngle;

    @Override
    public void runOpMode() {
        ElapsedTime timer = new ElapsedTime();
        HashMap<String, Boolean> flags = new HashMap<String, Boolean>();
        flags.put("vision", false);
        this.robot = new Robot(hardwareMap, telemetry, timer, AllianceColor.BLUE, gamepad1, gamepad2, flags);
        robot.drive.setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();
        while (opModeIsActive()) {
            robot.getGamePadInputs();
            if(gamepad1.a) {
                robot.control.extendBar(Control.BarState.HIGH);
            }
            if(gamepad1.b) {
                robot.control.extendBar(Control.BarState.MIDDLE);
            }
            if(gamepad1.y) {
                robot.control.extendBar(Control.BarState.LOW);
            }
//            telemetry.addData("Position (front left)", robot.frontLeftDriveMotor.getCurrentPosition());
//            telemetry.addData("Position (front right)", robot.frontRightDriveMotor.getCurrentPosition());
//            telemetry.addData("Position (rear left)", robot.rearLeftDriveMotor.getCurrentPosition());
//            telemetry.addData("Position (rear right)", robot.rearRightDriveMotor.getCurrentPosition());

            telemetry.addData("Position (bar)", robot.bar.getCurrentPosition());

            telemetry.update();
        }
    }


}
