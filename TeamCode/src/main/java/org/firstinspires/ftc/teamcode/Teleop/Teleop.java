package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Robot;
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
    private final int slowModePow = 5;

    private void initOpMode() throws IOException {
        // Initialize DC motor objects
        timer = new ElapsedTime();
        HashMap<String, Boolean> flags = new HashMap<>();
        flags.put("vision", true);
        this.robot =  new Robot(hardwareMap, telemetry, timer, AllianceColor.BLUE, gamepad1, gamepad2,flags);
        timeCurrent = timer.nanoseconds();
        timePre = timeCurrent;

        telemetry.addData("Waiting for start", "...");
        telemetry.update();
    }

    private void initDevices() {
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


        initDevices();

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        telemetry.clearAll();
        timeCurrent = timer.nanoseconds();
        timePre = timeCurrent;

        final double sensitivityHighPower = 1.0; // multiply inputs with this on high power mode
        final double sensitivityLowPower = 0.5; // multiply inputs with this on non-high power mode

        while (opModeIsActive()) { // clearer nomenclature for variables
            robot.getGamePadInputs();

            timeCurrent = timer.nanoseconds();
            deltaT = timeCurrent - timePre;
            timePre = timeCurrent;

            // Robot drive movement
            double[] motorPowers;

            if (driveHighPower) {
                motorPowers = robot.drive.calcMotorPowers(robot.leftStickX * sensitivityHighPower, robot.leftStickY * sensitivityHighPower, robot.rightStickX * sensitivityHighPower);
            }
            else {
                motorPowers = robot.drive.calcMotorPowers(robot.leftStickX * sensitivityLowPower, robot.leftStickY * sensitivityLowPower, robot.rightStickX * sensitivityLowPower);
            }
            robot.drive.setDrivePowers(motorPowers);

            //Toggle drive power
            if (robot.yButton && !robot.isyButtonPressedPrev){
                driveHighPower = !driveHighPower;
            }
        }
    }
}
