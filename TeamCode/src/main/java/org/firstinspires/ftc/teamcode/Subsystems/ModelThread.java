package org.firstinspires.ftc.teamcode.Subsystems;


import org.firstinspires.ftc.teamcode.DriveControl.BoundingBox;
import org.firstinspires.ftc.teamcode.Util.Coordinate;
import org.firstinspires.ftc.teamcode.Util.ExceptionHandler;
import org.tensorflow.lite.Interpreter;
import java.util.concurrent.locks.ReadWriteLock;
import java.util.concurrent.locks.ReentrantReadWriteLock;


public class ModelThread extends Thread {
    private VisionThreadData vtd = VisionThreadData.getVTD();
    private Interpreter modelInterpreter = new Interpreter(vtd.modelFile);
    ReadWriteLock lock = new ReentrantReadWriteLock();

    @Override
    public void run() {
        Thread.currentThread().setUncaughtExceptionHandler(new ExceptionHandler());
        lock.writeLock().tryLock();
        try {
            vtd.setDistance(calcDistanceFromBoundingBox(getBoundingBoxFromModel()));
        } finally {
            lock.writeLock().unlock();
        }
    }

    private double calcDistanceFromBoundingBox(BoundingBox box) {
        //TODO: Calculation code here
        return 0.0;
    }

    private BoundingBox getBoundingBoxFromModel() {
        return new BoundingBox(0d, 1d, 2d, new Coordinate(0, 0)); //TODO: Add code for obtaining usable bounding boxes from TF model, delete this filler when complete
    }
}
