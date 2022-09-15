package org.firstinspires.ftc.teamcode.PurePursuit;

import org.firstinspires.ftc.teamcode.Util.Coordinate;

import java.util.ArrayList;

public class GameObject {
    private ArrayList<Coordinate> occupies;
    public GameObject() {
        occupies = new ArrayList<>();
    }

    public GameObject(ArrayList<Coordinate> occupies) {
        this.occupies = occupies;
    }

    public void addCoordinate(Coordinate coordinate) {
        occupies.add(coordinate);
    }

    public ArrayList<Coordinate> getCoordinates() {
        return occupies;
    }

    public boolean occupies(Coordinate coordinate) {
        return this.occupies.contains(coordinate);
    }
}
