package org.firstinspires.ftc.teamcode.Subsystems.Drive;

import org.firstinspires.ftc.teamcode.Util.Vector;

public class Pose {
    public double angle;
    public Vector position;
    public double heading;
    public Pose(Vector position, double angle, double heading) {
        this.position = position;
        this.angle = angle;
        this.heading = heading;
    }
}
