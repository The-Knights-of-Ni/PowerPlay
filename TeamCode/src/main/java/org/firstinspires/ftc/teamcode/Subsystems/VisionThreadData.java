package org.firstinspires.ftc.teamcode.Subsystems;

import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.firstinspires.ftc.teamcode.DriveControl.BoundingBox;

import java.io.File;

public class VisionThreadData {
    private static VisionThreadData vtd;
    public static final File modelFile = new File(""); //TODO: Set this to the TensorFlow model file when trained.
    private static Vector2D distance;

    public static BoundingBox theoreticalPosition;

    public static VisionThreadData getVTD() {
        return vtd;
    }

    public static Vector2D getDistance() {
        return distance;
    }

    public void setDistance(Vector2D distance) {
        vtd.distance = distance;
    }
}
