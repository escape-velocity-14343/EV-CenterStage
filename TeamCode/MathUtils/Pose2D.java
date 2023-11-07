package org.firstinspires.ftc.teamcode.MathUtils;

/**
 * Stores position and angle of an object in 2D.
 * {X, Y, Angle}.
 */
public class Pose2D {
    public Vector vals;
    // double x;
    // double y;
    // double angle;

    public Pose2D(double _x, double _y, double _angle) {
        vals = new Vector(new double[]{_x, _y, Angle.normalize(_angle)});
        // x = _x;
        // y = _y;
        // angle = Angle.normalize(_angle);
    }

    public Pose2D(double[] _vals) {
        assert _vals.length == 3;
        vals = new Vector(_vals);
    }

    public Pose2D(Vector _vals) {
        assert _vals.size == 3;
        vals = _vals;
    }

    public Vector getPosition() {
        return new Vector(new double[]{vals.get(0), vals.get(1)});
    }

    public double getAngle() {
        return vals.get(2);
    }

    public static double getPositionDist(Pose2D pose1, Pose2D pose2) {
        return Vector.dist(new Vector(new double[]{pose1.vals.get(0), pose1.vals.get(1)}),
         new Vector(new double[]{pose2.vals.get(0), pose2.vals.get(1)}));
    }

    public static double getAngleDist(Pose2D pose1, Pose2D pose2) {
        return Angle.normalize(pose1.vals.get(2)-pose2.vals.get(2));
    }
}
