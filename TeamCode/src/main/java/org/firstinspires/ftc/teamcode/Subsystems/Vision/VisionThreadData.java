package org.firstinspires.ftc.teamcode.Subsystems.Vision;

import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.firstinspires.ftc.teamcode.DriveControl.BoundingBox;

import java.io.File;

public class VisionThreadData {
    private static VisionThreadData vtd;
    public static BoundingBox actualPosition;
    private static boolean close = false;

    public static VisionThreadData getVTD() {
        return vtd;
    }

    public static BoundingBox getActualPosition() {
        return actualPosition;
    }

    public void setActualPosition(BoundingBox actualPosition) {
        vtd.actualPosition = actualPosition;
    }

    public boolean getClose() {
        return close;
    }

    public void setClose(boolean newClose) {
        vtd.close = newClose;
    }
}
