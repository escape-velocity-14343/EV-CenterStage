package org.firstinspires.ftc.teamcode.subsystems.Controllers;

import org.firstinspires.ftc.teamcode.subsystems.MathUtils.Pose2D;

public class LockRobotPose {
    Pose2D lockPose;
    MichaelPID positionPID;
    MichaelPID rotationPID;

    Pose2D lastPose;
    Pose2D accelVals;


    public LockRobotPose(Pose2D pose, MichaelPID _positionPID, MichaelPID _rotationPID) {
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
