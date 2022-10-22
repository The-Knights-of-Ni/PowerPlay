package org.firstinspires.ftc.teamcode.Subsystems.Vision;


import android.util.Log;
import org.apache.commons.geometry.euclidean.twod.ConvexArea;
import org.apache.commons.geometry.euclidean.twod.Vector2D;
import org.apache.commons.geometry.euclidean.twod.path.LinePath;
import org.apache.commons.numbers.core.Precision;
import org.firstinspires.ftc.teamcode.DriveControl.BoundingBox;
import org.firstinspires.ftc.teamcode.Util.ThreadExceptionHandler;
import org.firstinspires.ftc.teamcode.Util.Vector;
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

    private Vector calcDistanceFromBoundingBox(BoundingBox currentPosition, BoundingBox theoreticalPosition) {
        return currentPosition.transformTo(theoreticalPosition);
    }

    private BoundingBox getBoundingBoxFromModel() {
        Precision.DoubleEquivalence precision = Precision.doubleEquivalenceOfEpsilon(1e-6);

        // create a connected sequence of line segments forming the unit square
        LinePath path = LinePath.builder(precision)
                .append(Vector2D.ZERO)
                .append(Vector2D.Unit.PLUS_X)
                .append(Vector2D.of(1, 1))
                .append(Vector2D.Unit.PLUS_Y)
                .build(true); // build the path, ending it with the starting point
        ConvexArea shape = ConvexArea.convexPolygonFromPath(path);
        return new BoundingBox(shape); //TODO: Add code for obtaining usable bounding boxes from TF model, delete this filler when complete
    }
}
