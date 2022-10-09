package org.firstinspires.ftc.teamcode.Subsystems;


import org.firstinspires.ftc.teamcode.DriveControl.BoundingBox;
import org.firstinspires.ftc.teamcode.Util.Coordinate;
import org.tensorflow.lite.Interpreter;

public class ModelThread extends Thread {
    private VisionThreadData vtd = VisionThreadData.getVTD();
    private Interpreter modelInterpreter = new Interpreter(vtd.modelFile);

    @Override
    public void run() {
        vtd.setDistance(calcDistanceFromBoundingBox(getBoundingBoxFromModel()));
    }

    private double calcDistanceFromBoundingBox(BoundingBox box) {
        //TODO: Calculation code here
        return 0.0;
    }

    private BoundingBox getBoundingBoxFromModel() {
        return new BoundingBox(0d, 1d, 2d, new Coordinate(0, 0)); //TODO: Add code for obtaining usable bounding boxes from TF model, delete this filler when complete
    }
}
