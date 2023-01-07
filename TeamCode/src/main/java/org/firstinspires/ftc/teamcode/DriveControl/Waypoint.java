package org.firstinspires.ftc.teamcode.DriveControl;

import org.firstinspires.ftc.teamcode.Util.Coordinate;

public class Waypoint {
    public final boolean fullStop;
    public Coordinate coordinate;

    public Waypoint(Coordinate c, boolean stop) {
        coordinate = c;
        fullStop = stop;
    }

}
