package org.firstinspires.ftc.teamcode.Trajectory;

import org.firstinspires.ftc.teamcode.MathUtils.Vector;

public abstract class Spline {

    /**
     * Length in terms of t value. Range for t is [0, length].
     */
    public double length = 1.0;

    /**
     * Arclength.
     */
    public double arclength;

    /**
     * @return t in [0, 1].
     */
    public double constrain(double t) {
        // code golf????
        return (t<0?0:(t>length?length:t))/length;
    }

    public abstract Vector getValue(double t);

    public abstract Vector getVelocity(double t);

    public abstract Vector getAcceleration(double t);

    /**
     * This will shift the t range to arclength parameterization ([0, arclen]).
     */
    public abstract void initArclen(int acc);

    public static double projectPos(Vector pos, Spline spline, double tolerance, int initialpoints) {
        double[] currvals = new double[initialpoints];
        Vector[] currvecs = new Vector[initialpoints];
        double mindist = Double.POSITIVE_INFINITY;
        int ind = 0;
        double dist = 0;
        for (int i = 0; i < initialpoints; i++) {
            currvals[i] = (i/((double) initialpoints-1))*spline.length;
            currvecs[i] = spline.getValue(currvals[i]);
            dist = Vector.dist(currvecs[i], pos);
            if (dist < mindist) {
                ind = i;
                mindist = dist;
            }
        }

        double curr = currvals[ind];
        //System.out.println(curr);

        for(int i = 0; i < 250; i++){
            Vector p = spline.getValue(curr);
            Vector deriv = spline.getVelocity(curr);

            double ds = Vector.dot2D(Vector.add(pos, p.scale(-1)), deriv);

            ds = ds / deriv.magnitude();
            
            if(-tolerance <= ds && ds <= tolerance) {
                break;
            }

            curr += ds / spline.length;

            curr = spline.constrain(curr)*spline.length;
            System.out.println(curr);
        }

        return spline.constrain(curr)*spline.length;
    }

    public static double getCurvature(double t, Spline spline) {
        t = spline.constrain(t);
        Vector velo = spline.getVelocity(t);
        Vector accel = spline.getAcceleration(t);
        double xd1 = velo.get(0);
        double yd1 = velo.get(1);
        double xd2 = accel.get(0);
        double yd2 = accel.get(1);
        return Math.pow(xd1*xd1+yd1*yd1, 1.5)/(xd1*yd2-xd2*yd1);
    }

}
