package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Subsystems.Control.*;
import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Util.AllianceColor;

@Autonomous(name = "Auto Blue Secondary Cycle", group = "Auto Blue")
public class AutoBlueSecondaryCycle extends Auto {
    @Override
    public void runOpMode() throws InterruptedException {
        initAuto(AllianceColor.BLUE);
        PlacementLevel placementLevel = getHubLevel();
        waitForStart();
        Drive drive = robot.drive;

        robot.control.setLidPosition(LidPosition.CLOSED);

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
        drive.turnByAngle(-30);
        drive.moveForward((21.5 + adjustment) * mmPerInch);

        // Release clamp
        robot.control.setLidPosition(LidPosition.DEPLOYED);
        sleep(500);
        drive.moveBackward((21.5 + adjustment) * mmPerInch);

        robot.control.setLidPosition(LidPosition.CLOSED);
        robot.control.setSlide(SlideState.RETRACTED);

        // Move back to the warehouse
        drive.turnByAngle(-60);
        drive.moveRight(5 * mmPerInch);
        robot.control.setLidPosition(LidPosition.OPEN);

        robot.control.setIntakeDirection(true, true);
        drive.moveBackward(32 * mmPerInch);

        for(int i = 0; i < 2; i++) {
            forwardCycle(i);
            backCycle(i);
        }

        // Ready devices for teleop
        robot.control.setIntakeDirection(false, false);
        robot.control.setLidPosition(LidPosition.OPEN);
        sleep(1000);

        telemetry.addLine("Done");
        telemetry.update();
    }

    public void backCycle(int i) {
        robot.drive.moveBackward(4 * mmPerInch);
        robot.control.setLidPosition(LidPosition.CLOSED);
        robot.control.setSlide(SlideState.RETRACTED);
        robot.drive.moveBackward(20 * mmPerInch);

        robot.drive.turnByAngle(-60);
        robot.drive.moveRight(6 * mmPerInch);
        robot.control.setIntakeDirection(true, true);
        robot.control.setLidPosition(LidPosition.OPEN);

        robot.drive.moveBackward((40)*mmPerInch);
    }
    public void forwardCycle(int i) {
        sleep(250);
        robot.control.setBucketState(BucketState.RAISED);
        robot.drive.moveForward((36 + i * 8)*mmPerInch);
        robot.control.setLidPosition(LidPosition.CLOSED);
        robot.control.setIntakeDirection(true, false);
        robot.control.setBucketState(BucketState.LEVEL);
        robot.control.setSlide(SlideState.TOP);

        robot.drive.moveLeft(4 * mmPerInch);
        robot.drive.turnByAngle(60);
        robot.control.setIntakeDirection(false, false);
        robot.drive.moveForward(24*mmPerInch);
        robot.control.setLidPosition(LidPosition.DEPLOYED);
        sleep(500);
    }
}