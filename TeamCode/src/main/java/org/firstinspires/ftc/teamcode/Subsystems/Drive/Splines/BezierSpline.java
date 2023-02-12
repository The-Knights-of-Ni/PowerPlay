package org.firstinspires.ftc.teamcode.Subsystems.Drive.Splines;

import org.firstinspires.ftc.teamcode.Util.Vector;

public class BezierSpline extends Spline {

    public BezierSpline(Vector target, int divisions) {
        super(target, divisions);
    }
    private double b(double t, double c2, double c3, double c4) {
        return Math.pow((1-t), 3.0)*0 + 3*t*Math.pow(1-t, 2.0)*c2 + 3*Math.pow(t, 2.0)*(1-t)*c3 + Math.pow(t, 3.0)*c4;
    }

    @Override
    protected double cX(double t) {
        return b(t, 0.25*target.getX(), 0.9*target.getX(), target.getX());
    }
    @Override
    protected double cY(double t) {
        return b(t, 0.1*target.getY(), 0.1*target.getY(), target.getY());
    }
}
