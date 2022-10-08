package org.firstinspires.ftc.teamcode.DriveControl;

import org.firstinspires.ftc.teamcode.Util.Coordinate;

import java.util.ArrayList;

public class GameObject {
    private final BoundingBox size;

    public GameObject(BoundingBox box) {
        this.size = box;
    }

    public boolean occupies(Coordinate coordinate) {
        return size.contains(coordinate);
    }
}
