package org.firstinspires.ftc.teamcode.Subsystems;

import org.firstinspires.ftc.teamcode.DriveControl.BoundingBox;

public class ModelThread extends Thread {
    private Vision vision = Vision.getVision();
    private BoundingBox box;

    @Override
    public void run() {
        vision.distance = calcDistanceFromBoundingBox(box);
    }

    public double calcDistanceFromBoundingBox(BoundingBox box) {
        //TODO: Calculation code here
        return 0.0;
    }
}
