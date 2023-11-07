package org.firstinspires.ftc.teamcode.subsystems.Trajectory;

import org.firstinspires.ftc.teamcode.subsystems.MathUtils.Vector;

public class TrajectorySegment {
    public Spline poseSpline;
    public boolean endsTrajectory;
    public TrajectorySegmentEndCommand runOnEnd = null;
    public double length;

    public TrajectorySegment(Spline spline, boolean _endsTrajectory, TrajectorySegmentEndCommand onEnd) {
        poseSpline = spline;
        endsTrajectory = _endsTrajectory;
        runOnEnd = onEnd;
        length = spline.length;
    }

    public TrajectorySegment(Spline spline, boolean _endsTrajectory) {
        poseSpline = spline;
        endsTrajectory = _endsTrajectory;
        length = spline.length;
    }

    public TrajectorySegment(Spline spline) {
        poseSpline = spline;
        endsTrajectory = false;
        length = spline.length;
    }

    public void initArclen(int acc) {
        poseSpline.initArclen(acc);
        length = poseSpline.length;
    }

    public Vector getValue(double t) {
        return poseSpline.getValue(t);
    }

    public Vector getVelocity(double t) {
        return poseSpline.getVelocity(t);
    }

    public Vector getAcceleration(double t) {
        return poseSpline.getAcceleration(t);
    }

    public void onEnd() {
        if (runOnEnd != null) {
            runOnEnd.action();
        }
    }



}

