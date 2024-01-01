package org.firstinspires.ftc.teamcode.pathutils;

/**
 * FTC-standard trapezoidal motion profile.
 */
public class AsymmetricMotionProfile {
    private double maxAcceleration;
    private double maxDeceleration;
    private double maxVelocity;
    public AsymmetricMotionProfile(double maxAccel, double maxDecel, double maxVel) {
        this.maxAcceleration = maxAccel;
        this.maxDeceleration = maxDecel;
        this.maxVelocity = maxVel;
    }

    public VAPair get(double velocity, double distance) {
        // find if we can decelerate in time
        double timetodecel = velocity / maxDeceleration;
        // if we can't, decelerate
        if ((timetodecel * velocity / 2) > distance) {
            return new VAPair(velocity, -maxDeceleration);
        // if we are above maxvelocity go to max velocity
        } else if (velocity > maxVelocity) {
            return new VAPair(maxVelocity, 0);
        // otherwise accelerate
        } else {
            return new VAPair(velocity, maxAcceleration);
        }
    }
}
