package org.firstinspires.ftc.teamcode.MathUtils;

/**
 * Angle unit, in radians.
 */
public class Angle {
    public static double normalize(double angle) {
        angle %= (2*Math.PI);
        if (Math.abs(angle) > Math.PI) {
            angle -= Math.signum(angle)*2*Math.PI;
        }
        return angle;
    }
}
