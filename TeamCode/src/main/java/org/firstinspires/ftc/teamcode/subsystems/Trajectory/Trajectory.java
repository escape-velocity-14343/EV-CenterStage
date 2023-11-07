package org.firstinspires.ftc.teamcode.subsystems.Trajectory;

import java.util.ArrayList;

import org.firstinspires.ftc.teamcode.subsystems.MathUtils.Vector;

public class Trajectory {
    public ArrayList<TrajectorySegment> trajectorySegments;
    double[] tpsum;
    public double length;
    public int currIndex = 0;

    public Trajectory(ArrayList<TrajectorySegment> segments) {
        trajectorySegments = segments;
        initTPsum();
    }

    public void initArcLengthParameterization(int acc) {
        for (TrajectorySegment trajseg : trajectorySegments) {
            trajseg.initArclen(acc);
        }
        initTPsum();
    }

    private void initTPsum() {
        tpsum = new double[trajectorySegments.size() + 1];
        tpsum[0] = 0;
        for (int i = 1; i <= trajectorySegments.size(); i++) {
            tpsum[i] = tpsum[i-1] + trajectorySegments.get(i-1).length;
        }
        length = tpsum[tpsum.length-1];
    }

    public int getIndex(double t) {
        for (int i = 1; i < tpsum.length; i++) {
            if (tpsum[i]+0.00001 > t) {
                return i-1;
            }
        }
        throw new Error("T-value " + t + " is outside of the range [0, " + tpsum[tpsum.length-1] + "].");
    }

    /**
     * @param index The index of the curve the t value is for.
     */
    public double getRelativeTValue(double t, int index) {
        return trajectorySegments.get(index).poseSpline.constrain(t - tpsum[index]);
    }

    /**
     * Returns the value at the specified t-value. Assumes spline continuity as float behaviour makes for unstable endpoints.
     */
    public Vector getValue(double t) {
        int ind = getIndex(t);
        return trajectorySegments.get(ind).getValue(t-tpsum[ind+1]);
    }

    /**
     * Returns the velocity at the specified t-value. Assumes spline continuity as float behaviour makes for unstable endpoints.
     */
    public Vector getVelocity(double t) {
        int ind = getIndex(t);
        return trajectorySegments.get(ind).getVelocity(t-tpsum[ind+1]);
    }

    /**
     * Returns the acceleration at the specified t-value. Assumes spline continuity as float behaviour makes for unstable endpoints.
     */
    public Vector getAcceleration(double t) {
        int ind = getIndex(t);
        return trajectorySegments.get(ind).getAcceleration(t-tpsum[ind+1]);
    }

    public Vector getValue(double t, int index) {
        return trajectorySegments.get(index).getValue(t);
    }

    public Vector getVelocity(double t, int index) {
        return trajectorySegments.get(index).getVelocity(t);
    }

    public Vector getAcceleration(double t, int index) {
        return trajectorySegments.get(index).getAcceleration(t);
    }

    public void next() {
        if (currIndex == trajectorySegments.size() - 1) {
            return;
        } else {
            trajectorySegments.get(currIndex).onEnd();
            currIndex++;
        }
    }
}   
