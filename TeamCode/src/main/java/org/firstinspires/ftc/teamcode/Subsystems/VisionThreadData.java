package org.firstinspires.ftc.teamcode.Subsystems;

import java.io.File;

public class VisionThreadData {
    private static VisionThreadData vtd;
    public static final File modelFile = new File(""); //TODO: Set this to the TensorFlow model file when trained.
    private static double distance;

    public static VisionThreadData getVTD() {
        return vtd;
    }

    public static double getDistance() {
        return distance;
    }

    public void setDistance(double distance) {
        this.distance = distance;
    }
}
