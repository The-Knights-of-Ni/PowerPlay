package org.firstinspires.ftc.teamcode.Subsystems.Drive.Splines;

import org.firstinspires.ftc.teamcode.Util.Vector;

public class LinearSpline extends Spline {

    public LinearSpline(Vector target, int divisions) {
        super(target, divisions);
    }

    @Override
    protected double cX(double t) {
        return t/divisions*target.getX();
    }
    @Override
    protected double cY(double t) {
        return t/divisions*target.getY();
    }
}
