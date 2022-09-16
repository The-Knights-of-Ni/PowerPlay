package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Subsystems.Control;
import org.firstinspires.ftc.teamcode.Subsystems.Control.PlacementLevel;
import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Util.AllianceColor;

@Autonomous(name = "Auto Blue Secondary Park", group = "Auto Blue")
public class AutoBlueSecondaryPark extends Auto {
    @Override
    public void runOpMode() throws InterruptedException {
        initAuto(AllianceColor.BLUE);
        PlacementLevel placementLevel = getHubLevel();
        waitForStart();
        Drive drive = robot.drive;

        robot.control.setLidPosition(Control.LidPosition.CLOSED);

        if(placementLevel != PlacementLevel.NOT_FOUND) {
            telemetry.addData("Level", placementLevel);
            telemetry.update();
        }

        // Move to hub (and start ScoreThread)
        new ScoreThread(robot, placementLevel).start();
        double adjustment = 0;
        switch(placementLevel) {
            case BOTTOM:
                break;
            case MIDDLE:
            case TOP:
                adjustment = 4;
                break;
        }
        drive.moveForward(2 * mmPerInch);
        drive.turnByAngle(-32.5);
        drive.moveForward((21 + adjustment) * mmPerInch);

        // Release clamp
        robot.control.setLidPosition(Control.LidPosition.DEPLOYED);
        sleep(500);
        drive.moveBackward((21 + adjustment) * mmPerInch);

        robot.control.setLidPosition(Control.LidPosition.CLOSED);
        robot.control.setSlide(Control.SlideState.RETRACTED);

        // Move back to the warehouse
        drive.turnByAngle(-57.5);
        drive.moveRight(5 * mmPerInch);
        drive.moveBackward(28 * mmPerInch);
        drive.moveLeft(4 * mmPerInch);
        drive.turnByAngle(90);
        drive.moveForward(22 * mmPerInch);
        drive.moveLeft(24 * mmPerInch);

        // Ready devices for teleop
        robot.control.setIntakeDirection(false, false);
        robot.control.setLidPosition(Control.LidPosition.OPEN);
        sleep(3000);

        telemetry.addLine("Done");
        telemetry.update();
    }
}