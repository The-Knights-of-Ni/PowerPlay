package org.firstinspires.ftc.teamcode.Subsystems.Drive.Splines;

import org.firstinspires.ftc.teamcode.Util.Vector;

public class LogSpline extends Spline {

    public LogSpline(Vector target, int divisions) {
        super(target, divisions);
    }

    @Override
    protected double cX(double t) {
        return t/divisions*target.getX();
    }
    @Override
    protected double cY(double t) {
        double x = t/divisions;
        return Math.log(x+1)/Math.log(target.getX()+1);
    }
}
