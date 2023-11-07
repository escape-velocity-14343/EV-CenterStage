package org.firstinspires.ftc.teamcode.Controllers;

import org.firstinspires.ftc.teamcode.MathUtils.Pose2D;

public class LockRobotPose {
    Pose2D lockPose;
    PIDController positionPID;
    PIDController rotationPID;

    Pose2D lastPose;
    Pose2D accelVals;


    public LockRobotPose(Pose2D pose, PIDController _positionPID, PIDController _rotationPID) {
        lockPose = pose;
        positionPID = _positionPID;
        rotationPID = _rotationPID;
    }

    /**
     * Updates desired acceleration and angular acceleration.
     * @param currentPose The acceleration vector will be created based on the distance units you use.
     * @param millisElapsed Enter in milliseconds!
     */
    public void update(Pose2D currentPose, double millisElapsed) {
        double xerror = currentPose.vals.get(0)-lockPose.vals.get(0);
        double yerror = currentPose.vals.get(1)-lockPose.vals.get(1);
        double roterror = currentPose.vals.get(2)-lockPose.vals.get(2);
        xerror = positionPID.get(xerror, millisElapsed);
        yerror = positionPID.get(yerror, millisElapsed);
        roterror = positionPID.get(roterror, millisElapsed);
        accelVals = new Pose2D(xerror, yerror, roterror);
    }

    /**
     * @return A pose of the desired acceleration and angular acceleration.
     */
    public Pose2D get() {
        return accelVals;
    }
}
