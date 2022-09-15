package org.firstinspires.ftc.teamcode.Util;

public class Coordinate {
    public double x;
    public double y;

    public Coordinate(int x, int y) {
        this.x = x;
        this.y = y;
    }
    public Coordinate(double x, double y) {
        this.x = x;
        this.y = y;
    }
    public boolean equals(Coordinate c) {
        return (this.x == c.x && this.y == c.y);
    }

    public String toString() {
        return "(" + this.x + "," + this.y + ")";
    }
}
