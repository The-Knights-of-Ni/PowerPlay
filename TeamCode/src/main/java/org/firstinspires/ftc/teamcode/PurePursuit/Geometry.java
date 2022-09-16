package org.firstinspires.ftc.teamcode.PurePursuit;

import org.firstinspires.ftc.teamcode.Util.Coordinate;

public class Geometry {
    public static double distance(Coordinate first, Coordinate second) {
        return Math.sqrt(Math.pow((second.x - first.x), 2) + Math.pow((second.y - first.y), 2));
    }
    /**
     * Returns closest point on segment to point
     * @param ss - segment start point
     * @param se - segment end point
     * @param p - point to found the closest point on segment
     * @return closest point on segment to p
     */
    public static Coordinate getClosestPointOnSegment(Coordinate ss, Coordinate se, Coordinate p) {
        return getClosestPointOnSegment(ss.x, ss.y, se.x, se.y, p.x, p.y);
    }

    /**
     * Returns closest point on segment to point
     * @param sx1 - segment x coord 1
     * @param sy1 - segment y coord 1
     * @param sx2 - segment x coord 2
     * @param sy2 - segment y coord 2
     * @param px - point x coord
     * @param py - point y coord
     * @return closets point on segment to point
     */
    public static Coordinate getClosestPointOnSegment(double sx1, double sy1, double sx2, double sy2, double px, double py) {
        double xDelta = sx2 - sx1;
        double yDelta = sy2 - sy1;

        if ((xDelta == 0) && (yDelta == 0)) {
            throw new IllegalArgumentException("Segment start equals segment end");
        }

        double u = ((px - sx1) * xDelta + (py - sy1) * yDelta) / (xDelta * xDelta + yDelta * yDelta);

        final Coordinate closestPoint;
        if (u < 0) {
            closestPoint = new Coordinate(sx1, sy1);
        }
        else if (u > 1) {
            closestPoint = new Coordinate(sx2, sy2);
        }
        else {
            closestPoint = new Coordinate((int) Math.round(sx1 + u * xDelta), (int) Math.round(sy1 + u * yDelta));
        }
        return closestPoint;
    }


    /**
     * Get the angle with the startpoint and endpoint being interchangeable.
     * @param startpoint
     * @param vertex
     * @param endpoint
     * @return
     */
    public static double getAngle(Coordinate startpoint, Coordinate vertex, Coordinate endpoint) {
        double a = distance(startpoint, vertex);
        double c = distance(startpoint, endpoint);
        double b = distance(vertex, endpoint);
        double numerator = Math.pow(a,2)+Math.pow(b, 2)-Math.pow(c,2);
        return Math.acos(numerator/(2*a*b));
    }
}