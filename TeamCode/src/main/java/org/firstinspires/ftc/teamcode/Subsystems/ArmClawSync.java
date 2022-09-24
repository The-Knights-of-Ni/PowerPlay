//package org.firstinspires.ftc.teamcode.Subsystems;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.DistanceSensor;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.Robot;
//
///**
// * Created by Paige Yeung on 01/02/20.
// */
//
//@TeleOp(name = "Arm-Claw Synchronization Test")
//public class ArmClawSync extends LinearOpMode {
//    //Declare DC motor objects
//    private Robot robot;
//
//    double leftStickX;
//    double leftStickY;
//    double rightStickX;
//    boolean aButton;
//    boolean bButton;
//    boolean dPadUp;
//    boolean dPadDown;
//    boolean dPadLeft;
//    boolean dPadRight;
//
//    double leftStickX2;
//    double leftStickY2;
//    double rightStickX2;
//    double rightStickY2;
//    boolean aButton2;
//    boolean bButton2;
//    boolean dPadUp2;
//    boolean dPadDown2;
//    boolean dPadLeft2;
//    boolean dPadRight2;
//    boolean bumperLeft2;
//    boolean bumperRight2;
//
//    double timePre;
//    double timeCurrent;
//    ElapsedTime timer;
//
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        initOpMode();
//        waitForStart();
//
//        //set foundation servo positions
//        robot.fClawL.setPosition(0.5);
//        robot.fClawR.setPosition(0.5);
//
//        double tgtPower = 0;
//
//        telemetry.clearAll();
//        while(opModeIsActive()) {
//            //Get gamepad inputs
//            leftStickX = gamepad1.left_stick_x;
//            leftStickY = -gamepad1.left_stick_y;
//            rightStickX = gamepad1.right_stick_x;
//            aButton = gamepad1.a;
//            bButton = gamepad1.b;
//            dPadUp = gamepad1.dpad_up;
//            dPadDown = gamepad1.dpad_down;
//            dPadLeft = gamepad1.dpad_left;
//            dPadRight = gamepad1.dpad_right;
//
//            leftStickX2 = gamepad2.left_stick_x;
//            leftStickY2 = -gamepad2.left_stick_y;
//            rightStickX2 = gamepad2.right_stick_x;
//            rightStickY2 = -gamepad2.right_stick_y;
//            aButton2 = gamepad2.a;
//            bButton2 = gamepad2.b;
//            dPadUp2 = gamepad2.dpad_up;
//            dPadDown2 = gamepad2.dpad_down;
//            dPadLeft2 = gamepad2.dpad_left;
//            dPadRight2 = gamepad2.dpad_right;
//            bumperLeft2 = gamepad2.left_bumper;
//            bumperRight2 = gamepad2.right_bumper;
//
//
//            timeCurrent = timer.nanoseconds();
//
//            if(aButton) {
//                robot.mainClaw.setPosition(robot.mainClaw.getPosition()+0.01);
//            }
//            if(bButton) {
//                robot.mainClaw.setPosition(robot.mainClaw.getPosition()-0.01);
//            }
//
//            if(aButton2) {
//                robot.mainArm.setPosition(robot.mainClaw.getPosition()+0.01);
//            }
//            if(bButton2) {
//                robot.mainArm.setPosition(robot.mainClaw.getPosition()-0.01);
//            }
//
//            telemetry.addData("Main Claw Servo Position", robot.mainClaw.getPosition());
//            telemetry.addData("Status", "Running");
//
//            telemetry.addData("Main Arm Servo Position", robot.mainArm.getPosition());
//            telemetry.addData("Status", "Running");
//
//            telemetry.addData("Difference", robot.mainClaw.getPosition()-robot.mainArm.getPosition());
//
//            telemetry.update();
//            timePre = timeCurrent;
//        }
//    }
//    private void initOpMode() {
//        //Initialize DC motor objects
//        timer = new ElapsedTime();
//        robot = new Robot(this, timer);
//        robot.xRailWinch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.xRailWinch.setTargetPosition(0);
//        robot.xRailWinch.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        robot.armTilt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.armTilt.setTargetPosition(0);
//        robot.armTilt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        timeCurrent = timer.nanoseconds();
//        timePre = timeCurrent;
//
//        telemetry.addData("Wait for start", "");
//        telemetry.update();
//    }
