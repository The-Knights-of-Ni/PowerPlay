package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Subsystems.Control.PlacementLevel;
import org.firstinspires.ftc.teamcode.Subsystems.DetectMarkerPipeline;
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
            stop();
        }
    }

    /**
     * Returns the hub level by running until the hub level is not {@link DetectMarkerPipeline.MarkerLocation#NOT_FOUND}
     * @return The hub level as an int.
     */
    public PlacementLevel getHubLevel() {
        PlacementLevel level;
        DetectMarkerPipeline.MarkerLocation location;
        do {
            location = robot.vision.detectMarkerRun();
        } while (location == DetectMarkerPipeline.MarkerLocation.NOT_FOUND);
        switch(location) {
            case LEFT:
                level = PlacementLevel.BOTTOM;
                break;
            case MIDDLE:
                level = PlacementLevel.MIDDLE;
                break;
            case RIGHT:
                level = PlacementLevel.TOP;
                break;
            default:
                level = PlacementLevel.NOT_FOUND;
                break;
        }
        robot.vision.stop(); // DO NOT remove this line. It is to avoid potential memory leaks
        return level;
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
