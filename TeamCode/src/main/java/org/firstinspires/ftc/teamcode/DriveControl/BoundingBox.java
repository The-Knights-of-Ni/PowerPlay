package org.firstinspires.ftc.teamcode.DriveControl;

import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.firstinspires.ftc.teamcode.Util.Coordinate;
import org.opencv.core.Core;

public class BoundingBox {
    public final double x;
    public final double y;
    public final double z;

    public Coordinate absoluteBottomLeft;

    /**
     * The bounding box generates a box to represent an objects position.
     * @param xc The length of the box in the x direction
     * @param yc The length of the box in the y direction
     * @param zc the height of the box
     * @param position The bottom left corner of the box's position, this grounds the box to a position
     */
    public BoundingBox(double xc, double yc, double zc, Coordinate position) {
        x = xc;
        y = yc;
        z = zc;
        absoluteBottomLeft = position;
    }

    public boolean contains(Coordinate coordinate) {
        if ((coordinate.x <= absoluteBottomLeft.x) || (coordinate.y <= absoluteBottomLeft.y)) {
            return false;
        }
        else return (!(coordinate.x >= (absoluteBottomLeft.x + x))) && (!(coordinate.y >= (absoluteBottomLeft.y + y)));
    }

    public Vector2D distanceFrom(BoundingBox other) {
        double xDistance = Math.abs(other.absoluteBottomLeft.x - absoluteBottomLeft.x);
        double yDistance = Math.abs(other.absoluteBottomLeft.y - absoluteBottomLeft.y);
        return new Vector2D(xDistance, yDistance);
    }
}
