package org.firstinspires.ftc.teamcode.Auto;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Subsystems.Control.LidPosition;
import org.firstinspires.ftc.teamcode.Subsystems.Control.PlacementLevel;
import org.firstinspires.ftc.teamcode.Subsystems.Control.SlideState;

public class ScoreThread extends Thread{
    private final Robot robot;
    private final PlacementLevel placementLevel;

    public ScoreThread(Robot robot, PlacementLevel placementLevel) {
        this.robot = robot;
        this.placementLevel = placementLevel;
    }

    @Override
    public void run() {
        try {
//            robot.telemetryBroadcast("Thread Status", "Running");
            robot.control.setLidPosition(LidPosition.CLOSED);
            switch (placementLevel) {
                case TOP:
                    robot.control.setSlide(SlideState.TOP);
                    break;
                case MIDDLE:
                    robot.control.setSlide(SlideState.MIDDLE);
                    break;
                case BOTTOM:
                    robot.control.setSlide(SlideState.BOTTOM);
                    break;
            }
        } catch (Exception e) {
//            telemetry.addData("Thread Status", "Interrupted " + e);
        }
    }

}
