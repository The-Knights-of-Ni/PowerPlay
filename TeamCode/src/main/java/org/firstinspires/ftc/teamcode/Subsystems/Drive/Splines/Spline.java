package org.firstinspires.ftc.teamcode.Subsystems.Drive.Splines;

import org.firstinspires.ftc.teamcode.Subsystems.Drive.Pose;
import org.firstinspires.ftc.teamcode.Util.Vector;

import java.util.ArrayList;

public abstract class Spline {
    public final Vector target;
    protected final int divisions;
    public ArrayList<Pose> poses;
    public Spline(Vector target, int divisions) {
        this.target = target;
        this.divisions = divisions;
        poses = new ArrayList<>(divisions);
        calculate();
    }

    protected abstract double cX(double t);
    protected abstract double cY(double t);
    protected void calculate() {
        for (double i = 0; i <= 1.; i += (double) 1 / divisions) {
            poses.add(new Pose(new Vector(cX(i), cY(i)), 0, 0));
        }
        // ensure that the robot moves exactly where you want it to
        poses.remove(poses.size()-1); // Remove the last element
        poses.add(new Pose(new Vector(cX(target.getX()), cY(target.getY())), 0, 0)); // And add a proper one to account for floating point errors
        for (int i = 1; i < poses.size(); i++) {
            double xDiff = Math.abs(poses.get(i).position.getX() - poses.get(i-1).position.getX());
            double yDiff = Math.abs(poses.get(i).position.getY() - poses.get(i-1).position.getY());
            poses.get(i).heading = Math.atan2(xDiff, yDiff);
        }
    }
}
