package org.firstinspires.ftc.teamcode.PurePursuit;

import org.firstinspires.ftc.teamcode.Util.Coordinate;

public class RobotModel {
    private double circleRadius = 1.0;
    private double speed;
    private double angle;
    private Coordinate center;
    public RobotModel(Coordinate start) {
        center = start;
    }

    public void move(Coordinate position) {
        center = position;
    }

    public Coordinate getCenter() {
        return center;
    }

}
