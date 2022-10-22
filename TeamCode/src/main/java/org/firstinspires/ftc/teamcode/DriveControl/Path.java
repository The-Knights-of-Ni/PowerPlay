package org.firstinspires.ftc.teamcode.DriveControl;

import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.firstinspires.ftc.teamcode.Util.Coordinate;
import org.firstinspires.ftc.teamcode.Util.Vector;

import java.util.ArrayList;

public class Path {
    ArrayList<Waypoint> waypoints;
    int currentWaypoint;
    int targetWaypoint;


    private boolean isMoving;
    public Path(ArrayList<Waypoint> stops) {
        waypoints = stops;
        currentWaypoint = 0;
        targetWaypoint = 1;
        isMoving = false;
    }

    public Vector getVectorForPosition(Coordinate coordinate) {
        return new Vector(Math.abs(coordinate.getX() - waypoints.get(targetWaypoint).coordinate.getX()),
                Math.abs(coordinate.getY() - waypoints.get(targetWaypoint).coordinate.getY()));
    }
}
