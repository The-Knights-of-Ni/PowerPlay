package org.firstinspires.ftc.teamcode.Auto;

import android.util.Log;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Util.AllianceColor;

/**
 * Auto creates a robots and runs it in auto mode.
 *
 * <p>Auto currently just initializes the Robot as Auto.runOpMode() is empty.</p>
 * <p>Tasks:</p>
 * <p>Deliver duck from carousel (10)</p>
 *
 * <p>Deliver freight to hub (6)</p>
 *
 * <p>   - deliver freight to corresponding level of custom element (20)</p>
 *
 * <p>Park in warehouse (10)</p>
 *
 * @see com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
 */
@Autonomous(name = "Auto", group = "Concept")
@Disabled
public class Auto extends LinearOpMode {

    /**
     * Number of millimeters per an inch
     */
    public static final float mmPerInch = 25.4f;
    /**
     * The robot class in the op mode
     */
    public Robot robot;


    /**
     * Inits the robot class and sets the robot as the newly inited robot.
     * @param allianceColor The alliance color
     */
    // START ROBOT 4.5 INCHES AWAY FROM LEFT EDGE
    public void initAuto(AllianceColor allianceColor) {
        ElapsedTime timer = new ElapsedTime();
        try {
            this.robot = new Robot(this, hardwareMap, telemetry, timer, allianceColor, gamepad1, gamepad2,
                    true);
            telemetry.addData("Waiting for start", "");
            telemetry.update();
        } catch (Exception ioException) {
            telemetry.addData("ERROR: ", "IO EXCEPTION", ioException);
            telemetry.update();
            Log.e("main", "fatal error IO EXCEPTION", ioException);
            stop();
        }
    }

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
    }
}