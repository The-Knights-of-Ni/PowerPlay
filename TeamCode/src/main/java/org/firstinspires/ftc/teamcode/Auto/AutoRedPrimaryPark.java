package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Subsystems.Control.BucketState;
import org.firstinspires.ftc.teamcode.Subsystems.Control.LidPosition;
import org.firstinspires.ftc.teamcode.Subsystems.Control.PlacementLevel;
import org.firstinspires.ftc.teamcode.Subsystems.Control.SlideState;
import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Util.AllianceColor;

/**
 * Auto creates a robots and runs it in auto mode. This auto class is for when we are on the red
 * alliance.
 *
 * <p>Auto currently just initializes the Robot as Auto.runOpMode() is empty.
 *
 * @see com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
 */

// Tasks:
// Deliver duck from carousel (10)
// Deliver freight to hub (6)
// - deliver freight to corresponding level of custom element (20)
// Park in warehouse (10)
@Autonomous(name = "Auto Red Primary Park", group = "Auto Red")
public class AutoRedPrimaryPark extends Auto {
    /**
     * Override of {@link Auto#runOpMode()}
     *
     * <p>Please do not swallow the InterruptedException, as it is used in cases where the op mode
     * needs to be terminated early.
     *
     * @throws InterruptedException Thrown when the op mode needs to be terminated early.
     * @see com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
     */
    @Override
    public void runOpMode() throws InterruptedException {
        initAuto(AllianceColor.RED);
        PlacementLevel placementLevel = getHubLevel();
        waitForStart();
        Drive drive = robot.drive;

        if (placementLevel != PlacementLevel.NOT_FOUND) {
            telemetry.addData("Level", placementLevel);
            telemetry.update();
        }

        // Move to carousel
        robot.control.setBucketState(BucketState.LEVEL);
        robot.control.setLidPosition(LidPosition.CLOSED);
        drive.moveForward(2 * mmPerInch);
        drive.turnByAngle(-90);
        robot.control.startCarousel(false);
        drive.moveBackward(25 * mmPerInch);

        // Deliver Duck
        sleep(2500);
        robot.control.stopCarousel();

        // Move to hub (and start ScoreThread)
        new ScoreThread(robot, placementLevel).start();
        drive.moveForward(50 * mmPerInch);
        drive.turnByAngle(90);

        double adjustment = 0;
        switch(placementLevel) {
            case BOTTOM:
                break;
            case MIDDLE:
            case TOP:
                adjustment = 4;
                break;
        }
        drive.moveForward((15 + adjustment) * mmPerInch);

        // Release clamp
        robot.control.setLidPosition(LidPosition.DEPLOYED);
        sleep(500);

        // Move back to park
        drive.moveBackward((4 + adjustment) * mmPerInch);
        sleep(1000);
        drive.turnByAngle(-80);
        robot.control.setLidPosition(LidPosition.CLOSED);
        robot.control.setSlide(SlideState.RETRACTED);

        drive.moveBackward(50 * mmPerInch);
        drive.moveLeft(16 * mmPerInch);

        // Ready devices for teleop
        robot.control.setBucketState(BucketState.LEVEL);
        robot.control.setLidPosition(LidPosition.OPEN);
        sleep(3000);

        telemetry.addLine("Done");
        telemetry.update();
    }
}
