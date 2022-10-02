package org.firstinspires.ftc.teamcode.DriveControl;

import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.firstinspires.ftc.teamcode.Util.Coordinate;

public class RobotModel {
    public Vector2D momentum;
    public double angle;
    public BoundingBox boundingBox;
    public RobotModel(BoundingBox initialPosition, double initialAngle) {
        boundingBox = initialPosition;
        momentum = new Vector2D(0,0);
        angle = initialAngle;
    }
}
