package org.firstinspires.ftc.teamcode.Subsystems.Vision;

import org.firstinspires.ftc.teamcode.DriveControl.BoundingBox;
import org.firstinspires.ftc.teamcode.Util.Vector;

import java.io.File;

public class VisionThreadData {
    private static VisionThreadData vtd;
    public static final File modelFile = new File(""); // TODO: Set this to the TensorFlow model file when trained.
    private static Vector distance;

    public static BoundingBox theoreticalPosition;

    public static VisionThreadData getVTD() {
        return vtd;
    }

    public static Vector getDistance() {
        return distance;
    }

    public void setDistance(Vector2D distance) {
        vtd.distance = distance;
    }
}
