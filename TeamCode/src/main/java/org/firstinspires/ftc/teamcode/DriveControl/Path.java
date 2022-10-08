package org.firstinspires.ftc.teamcode.DriveControl;

import org.firstinspires.ftc.teamcode.Util.Coordinate;

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

    public void getVectorForPosition(Coordinate coordinate) {

    }
}
