package org.firstinspires.ftc.teamcode.Subsystems.Vision;


import android.util.Log;
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.firstinspires.ftc.teamcode.DriveControl.BoundingBox;
import org.firstinspires.ftc.teamcode.Util.Coordinate;
import org.firstinspires.ftc.teamcode.Util.ThreadExceptionHandler;
import org.tensorflow.lite.Interpreter;
import java.util.concurrent.locks.ReadWriteLock;
import java.util.concurrent.locks.ReentrantReadWriteLock;


public class ModelThread extends Thread {
    private VisionThreadData vtd = VisionThreadData.getVTD();
    private Interpreter modelInterpreter = new Interpreter(vtd.modelFile);
    ReadWriteLock lock = new ReentrantReadWriteLock();

    @Override
    public void run() {
        Thread.currentThread().setUncaughtExceptionHandler(new ThreadExceptionHandler());
        if (lock.writeLock().tryLock()) {
            try {
                vtd.setDistance(calcDistanceFromBoundingBox(getBoundingBoxFromModel(), vtd.theoreticalPosition));
            } finally {
                lock.writeLock().unlock();
            }
        } else {
            Log.e("vision", "unable to acquire lock");
        }
    }

    private Vector2D calcDistanceFromBoundingBox(BoundingBox currentPosition, BoundingBox theoreticalPosition) {
        return currentPosition.distanceFrom(theoreticalPosition);
    }

    private BoundingBox getBoundingBoxFromModel() {
        return new BoundingBox(18d, 18d, 18d, new Coordinate(0, 0)); //TODO: Add code for obtaining usable bounding boxes from TF model, delete this filler when complete
    }
}
