package org.firstinspires.ftc.teamcode.Auto;

import android.util.Log;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Subsystems.Drive.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.Vision.Vision;
import org.firstinspires.ftc.teamcode.Util.AllianceColor;

import java.util.HashMap;

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
public abstract class Auto extends LinearOpMode {

    /**
     * Number of millimeters per an inch
     */
    public static final float mmPerInch = (float) Drive.mmPerInch;
    /**
     * The robot class in the op mode
     */
    public Robot robot;
    public ElapsedTime timer;
    /**
     * Initializes the robot class and sets the robot as the newly initialized robot.
     * @param allianceColor The alliance color
     */
    public void initAuto(AllianceColor allianceColor) {
        Log.i("main", "*** Opmode control passed to teamcode ***");
        timer = new ElapsedTime();
        try {
            HashMap<String, Boolean> flags = new HashMap<>();
            flags.put("vision", true);
            flags.put("web", false);
            this.robot = new Robot(hardwareMap, telemetry, timer, allianceColor, gamepad1, gamepad2, flags);
            robot.control.initDevicesAuto();
            telemetry.addData("Waiting for start", "");
            telemetry.update();
        } catch (Exception ioException) {
            telemetry.addData("ERROR: ", "IO EXCEPTION", ioException);
            telemetry.update();
            Log.e("main", "fatal error IO EXCEPTION", ioException);
            stop();
        }
    }
}
