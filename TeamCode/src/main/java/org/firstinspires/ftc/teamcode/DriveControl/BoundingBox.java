package org.firstinspires.ftc.teamcode.DriveControl;

import org.firstinspires.ftc.teamcode.Util.Coordinate;
import org.opencv.core.Core;

public class BoundingBox {
    public final double x;
    public final double y;
    public final double z;

    public Coordinate absoluteBottomLeft;

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
}
