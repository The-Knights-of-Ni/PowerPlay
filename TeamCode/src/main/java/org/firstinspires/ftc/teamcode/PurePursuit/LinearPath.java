package org.firstinspires.ftc.teamcode.PurePursuit;

import org.firstinspires.ftc.teamcode.Util.Coordinate;

import java.util.ArrayList;

public class LinearPath {
    public ArrayList<Coordinate> path;

    private int current = 0;

    public LinearPath() {
        this.path = new ArrayList<>();
    }

    public LinearPath(ArrayList<Coordinate> path) {
        this.path = path;
    }

    public Coordinate next() {
        current ++;
        return path.get(current);
    }

    public void addCoordinate(Coordinate c) {
        path.add(c);
    }

    public void addCoordinate(int index, Coordinate c) {
        path.add(index, c);
    }

    public void addWaypoint(GameObject o) {
        ArrayList<Coordinate> reach = o.getCoordinates();
        int smallestIndex = 0;
        double smallestLength = 0;
        for (Coordinate c: reach) {
            double currentLength = Geometry.distance(path.get(path.size()-1), c);
            if (currentLength < smallestLength) {
                smallestIndex = reach.indexOf(c);
                smallestLength = currentLength;
            }
        }
        path.add(reach.get(smallestIndex));
    }

    public void reset(Coordinate coordinate) {
        current = path.indexOf(coordinate);
    }

    public double getAngle(Coordinate coordinate) {
        int place = path.indexOf(coordinate);
        return Geometry.getAngle(path.get(place-1), coordinate, path.get(place+1));
    }

    public int getCurrent() {
        return current;
    }
}
