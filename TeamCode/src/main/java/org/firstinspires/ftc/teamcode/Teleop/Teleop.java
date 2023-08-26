package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Subsystems.Control.Control;
import org.firstinspires.ftc.teamcode.Util.AllianceColor;

import java.io.IOException;
import java.util.HashMap;

@TeleOp(name = "Teleop")
public class Teleop extends LinearOpMode {
    double deltaT;
    double timeCurrent;
    double timePre;
    ElapsedTime timer;
    private Robot robot;

    private boolean driveHighPower = true;
    private boolean armDeployed = false;

    private void initOpMode() throws IOException {
        // Initialize DC motor objects
        timer = new ElapsedTime();
        HashMap<String, Boolean> flags = new HashMap<String, Boolean>();
        flags.put("vision", false);
        this.robot =  new Robot(hardwareMap, telemetry, timer, AllianceColor.BLUE, gamepad1, gamepad2,flags);
        timeCurrent = timer.nanoseconds();
        timePre = timeCurrent;

        telemetry.addData("Waiting for start", "...");
        telemetry.update();
    }

    /**
     * Override of runOpMode()
     *
     * <p>Please do not swallow the InterruptedException, as it is used in cases where the op mode
     * needs to be terminated early.
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
     */
    @Override
    public void runOpMode() throws InterruptedException {
        try {
            initOpMode();
        } catch (IOException e) {
            e.printStackTrace();
        }

        ElapsedTime timer = new ElapsedTime();
        robot.control.initDevicesTeleop();
        waitForStart();

        telemetry.clearAll();
        timeCurrent = timer.nanoseconds();
        timePre = timeCurrent;

        final double sensitivityHighPower = 1.0; // multiply inputs with this on high power mode
        final double sensitivityLowPower = 0.7; // multiply inputs with this on non-high power mode

        while (opModeIsActive()) { // clearer nomenclature for variables
            robot.gamepads.update();

            timeCurrent = timer.nanoseconds();
            deltaT = timeCurrent - timePre;
            timePre = timeCurrent;

            driveHighPower = robot.gamepads.yButton.toggle;

            if (robot.gamepads.bButton.isPressed()) {
                robot.bar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }

            if (robot.gamepads.aButton.isPressed() && !robot.gamepads.aButton.hasPressedPrev()) {
                if(armDeployed) {
                    robot.control.pickupArm();
                    armDeployed = false;
                } else {
                    robot.control.dropoffArm();
                    armDeployed = true;
                }
            }

            // Robot drive movement
            double[] motorPowers;
            if (driveHighPower) {
                motorPowers = robot.drive.calcMotorPowers(robot.gamepads.leftStickX * sensitivityHighPower, robot.gamepads.leftStickY * sensitivityHighPower, robot.gamepads.rightStickX * sensitivityHighPower);
            }
            else {
                motorPowers = robot.drive.calcMotorPowers(robot.gamepads.leftStickX * sensitivityLowPower, robot.gamepads.leftStickY * sensitivityLowPower, robot.gamepads.rightStickX * sensitivityLowPower);
            }
            robot.drive.setDrivePowers(motorPowers);

            // Score state control
            if (robot.gamepads.dPadUp.isPressed() || robot.gamepads.dPadUp2.isPressed()) {
                robot.control.deploy(Control.BarState.HIGH);
            }
            if (robot.gamepads.dPadLeft.isPressed() || robot.gamepads.dPadLeft2.isPressed()) {
                robot.control.deploy(Control.BarState.LOW);
            }
            if (robot.gamepads.dPadRight.isPressed() || robot.gamepads.dPadRight2.isPressed()) {
                robot.control.deploy(Control.BarState.MIDDLE);
            }
            if (robot.gamepads.dPadDown.isPressed() || robot.gamepads.dPadDown2.isPressed()) {
                robot.control.retract();
            }

            // Manual 4-bar override
            if (robot.gamepads.bumperRight.isPressed() || robot.gamepads.bumperRight2.isPressed()) {
                robot.bar.setPower(1);
                robot.bar.setTargetPosition(robot.bar.getCurrentPosition() + 150);
                robot.bar.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            }
            if (robot.gamepads.bumperLeft.isPressed() || robot.gamepads.bumperLeft2.isPressed()) {
                robot.bar.setPower(1);
                robot.bar.setTargetPosition(robot.bar.getCurrentPosition() - 150);
                robot.bar.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            }

            // Claw open-close
            if (robot.gamepads.triggerRight > 0.5) {
                robot.control.toggleClaw(Control.ClawState.OPEN);
            }
            if (robot.gamepads.triggerLeft > 0.5) {
                robot.control.toggleClaw(Control.ClawState.CLOSED);
            }

        }
    }
}
