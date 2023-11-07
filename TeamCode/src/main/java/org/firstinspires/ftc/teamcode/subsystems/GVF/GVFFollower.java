package org.firstinspires.ftc.teamcode.subsystems.GVF;


import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.util.InterpLUT;

import org.firstinspires.ftc.teamcode.subsystems.MathUtils.*;
import org.firstinspires.ftc.teamcode.subsystems.Trajectory.*;
import org.firstinspires.ftc.teamcode.subsystems.Controllers.*;

@Config
public class GVFFollower {

    double MIN_VELOCITY;
    double MAX_VELOCITY;
    double MAX_ACCEL;
    int NUM_WHEELS;
    double positionTolerance;
    double angleTolerance;

    double correctionMultiplier;
    double currentVelocityScalar;

    double lastAngleError;
    MichaelPID rotPID;

    double endOfPathFactor;

    double curvatureGain;
    double curvatureLookaheadVal;
    double[][] pathCurvature;
    double[] curvaturePsum;

    public Trajectory trajectory;
    public Trajectory headingTrajectory;

    public Pose2D lastPose;
    Vector velocity;
    double tval = 0;
    int currindex = 0;
    
    public Vector movementvect;
    public double rotationCorrection;

    public static double kStatic = 0.2;

    InterpLUT veloPowerLUT = new InterpLUT();
    InterpLUT angularVelocityPowerLUT = new InterpLUT();
    InterpLUT degreesToBrakeAngularVelocityLUT = new InterpLUT();

    // for future implementation
    //    double[] speeds;
    //    Vector[] wheelvects;

    boolean isTimeout = false;
    /**
     * True if and only if the robot has finished a trajectory.
     */
    public boolean isStopped = false;
    public void addLUT() {
        int[] speeds = {7,12,14,15,25,30,35,37,40,45,49,53,55,60,65,67,100,1000};

        for(int i = speeds.length - 1; i >= 0; i--) {
            veloPowerLUT.add(-speeds[i], -0.25+0.05*i);
        }
        veloPowerLUT.add(0,0.0);
        for (int i=0;i<speeds.length;i++) {
            veloPowerLUT.add(speeds[i],0.25+0.05*i);
        }
        veloPowerLUT.createLUT();
        double[] angularVeloc = {0   , 22.3, 28.2, 46.7, 86.2, 123, 158,  198,  228,  261,  290,  326, 356,   392, 432, 477,   496,  541,  556, 600, 9999999};
        double[] angularPower = {0.11, 0.12, 0.13, 0.15, 0.2, 0.25, 0.3, 0.35, 0.40, 0.45, 0.50, 0.55, 0.6, 0.65, 0.7, 0.75, 0.80, 0.85, 0.90, 0.95, 1.0};
        double[] angularBrakingDist={0,1.81, 2.44,  4.5,10.2, 18.9,29.9, 40.4, 55.8, 67.9, 79.9, 96.3, 116, 136 , 156,  177,  207, 216, 237  , 268 , 9999999};
        for(int i =angularPower.length -1; i>= 1;i--) {
            angularVelocityPowerLUT.add(-angularVeloc[i], -angularPower[i]);
        }
        for(int i =0; i < angularPower.length;i++) {
            angularVelocityPowerLUT.add(angularVeloc[i], angularPower[i]);
        }
        angularVelocityPowerLUT.createLUT();
        degreesToBrakeAngularVelocityLUT.add(-9999999, 0);
        for(int i = 0; i < angularVeloc.length; i++) {
            degreesToBrakeAngularVelocityLUT.add(angularBrakingDist[i], angularVeloc[i]);
        }
        degreesToBrakeAngularVelocityLUT.createLUT();
        //odometry.testFastTrig();
    }

    /**
     *
     * @param velocity
     * @return Power in range [-1, 1].
     */
    public double velocityToPower(double velocity) {
        return Math.signum(velocity)*veloPowerLUT.get(Math.abs(velocity));
    }

    /**
     * @param correctionScalar A linear constant applied to the vector that is orthagonal to the path.
     * @param currVeloScalar A linear constant applied to the current velocity vector. Use to correct for steady state error ONLY IF NEEDED, otherwise keep at 1.
     * @param headingPID The heading PID controller.
     * @param pathEndGain An exponential constant scaling how aggressively the robot slows down as it approaches the goal.
     * @param _curvatureGain An exponential constant scaling how aggressively the robot slows down as it enters areas of high curvature.
     * @param curvatureTval A constant scaling the range of the local curvature (i.e. how big of an "area" around the robot the curvature correction takes in). 
     */

    public GVFFollower(double correctionScalar, double currVeloScalar, MichaelPID headingPID, double pathEndGain, double _curvatureGain, double curvatureTval) {
        correctionMultiplier = correctionScalar;
        currentVelocityScalar = currVeloScalar;
        rotPID = headingPID;
        endOfPathFactor = pathEndGain;
        curvatureGain = _curvatureGain;
        curvatureLookaheadVal = curvatureTval;
        //movementvect = new Vector(0.0, 0.0);
    }

    /**
     * Use any distance unit you would like, but ensure that they are the same unit. YOUR VELOCITY OUTPUTS WILL COME BACK IN THIS SAME UNIT. All time units should be in seconds.
     * @param maxaccel The true max acceleration of your robot should be (Max change in velocity)/(seconds to change). You may use any value for maxaccel as long as it is within [0, true max accel].
     */
    public void initRobot(double minvelo, double maxvelo, double maxaccel, int numwheels) {
        MIN_VELOCITY = minvelo;
        MAX_VELOCITY = maxvelo;
        MAX_ACCEL = maxaccel;
        NUM_WHEELS = numwheels;
    }

    /**
     * All parameters should be in the same distance units as your velocity and acceleration.
     */
    public void initTrajectory(Trajectory trajectoryToFollow, Trajectory _headingTrajectory, double posTolerance, double angTolerance, int curvatureAcc, Pose2D poseEsimate) {
        trajectory = trajectoryToFollow;
        headingTrajectory = _headingTrajectory;
        positionTolerance = posTolerance;
        angleTolerance = angTolerance;
        lastPose = poseEsimate;
        lastAngleError = 0;
        initCurvature(curvatureAcc);
    }

    private void initCurvature(int acc) {
        double pathlen = trajectory.length;
        pathCurvature = new double[acc+1][2];
        curvaturePsum = new double[acc+2];
        curvaturePsum[0] = 0;
        for (int i = 0; i <= acc; i += 1) {
            double t = ((double)i)*pathlen/((double)acc);
            int index = trajectory.getIndex(t);   
            pathCurvature[i][0] = t;
            pathCurvature[i][1] = Spline.getCurvature(trajectory.getRelativeTValue(t, index), trajectory.trajectorySegments.get(index).poseSpline);
            curvaturePsum[i+1] = curvaturePsum[i] + pathCurvature[i][1];
        }
    }

    /**
     * Returns the vector that defines the intended movement of the drivetrain.
     */
    private Vector getMovementVector(Pose2D pose) {
        Vector position = pose.getPosition();
        
        Vector projpoint = trajectory.getValue(tval, currindex);
        Vector endpoint = trajectory.getValue(trajectory.trajectorySegments.get(currindex).length, currindex);

        Vector movementvect = Vector.unitVector2D(trajectory.getVelocity(tval, currindex).arg);

        // THIS CODE DOES NOT WORK.
        // verify the above please

        // if we are close to the end, the movement vector should be the projected end vector onto the movement vector (this allows pathfinding from past the path as well)
        if (tval >= trajectory.trajectorySegments.get(currindex).length - 5*MAX_ACCEL/correctionMultiplier) {
            double scalar = Vector.dot2D(movementvect, Vector.add(endpoint, position.scale(-1)))/movementvect.magnitude();
            if (scalar < 1) {
                movementvect = movementvect.scale(scalar);
            }
        }

        Vector correctionvect = Vector.add(position.scale(-1), projpoint).scale(correctionMultiplier);
        Log.println(Log.INFO, "GVF", "Correction vector coordinates: " + correctionvect.get(0) + ", " + correctionvect.get(1));

        Vector mainvect = Vector.unitVector2D(Vector.add(correctionvect, movementvect).arg).scale(MAX_VELOCITY);
        Log.println(Log.INFO, "GVF", "Main vector coordinates: " + mainvect.get(0) + ", " + mainvect.get(1));

        // movement vector punitive multiplier
        double correctionFactor = 0;
        if (trajectory.trajectorySegments.get(currindex).endsTrajectory) {
            correctionFactor += endOfPathFactor/Vector.dist(position, endpoint);
        }

        


        // correcting for current velocity * gain
        Log.println(Log.INFO, "GVF", "Correction Factor: " + Math.exp(-correctionFactor));
        mainvect = mainvect.scale(Math.exp(-correctionFactor));
        mainvect = Vector.add(mainvect, velocity.scale(-currentVelocityScalar));

        double scalar = mainvect.magnitude();

        if (scalar > MAX_ACCEL) {
            mainvect.scale(MAX_ACCEL/scalar);
        }

        Log.println(Log.INFO, "GVF", "Final vector coordinates: " + mainvect.get(0) + ", " + mainvect.get(1));

        return mainvect;
    }

    /**
     * Handles end of segment/end of trajectory conditions.
     */
    private void updateTrajectory() {
        tval = Spline.projectPos(lastPose.getPosition(), trajectory.trajectorySegments.get(trajectory.currIndex).poseSpline, 1.0, 25);
        Log.println(Log.INFO, "GVF", "Distance to end: " + Vector.dist(lastPose.getPosition(), trajectory.getValue(trajectory.trajectorySegments.get(currindex).length, currindex)));
        Log.println(Log.INFO, "GVF", "tval: " + tval);
        Log.println(Log.INFO, "GVF", "angle dist: " + Angle.normalize(Math.abs(headingTrajectory.getValue(tval, currindex).get(0)-lastPose.getAngle())));
        if (Vector.dist(lastPose.getPosition(), trajectory.getValue(trajectory.trajectorySegments.get(currindex).length, currindex)) <= positionTolerance &&
            Angle.normalize(Math.abs(headingTrajectory.getValue(tval, currindex).get(0)-lastPose.getAngle())) <= angleTolerance) {
                if (currindex == trajectory.trajectorySegments.size() - 1) {
                    Log.println(Log.WARN, "GVF", "path is ending and this shouldnt be happening >:(");
                    currindex--;
                    stop();
                }
                Log.println(Log.WARN, "GVF", "Switching Spline Segments.");
                trajectory.next();
                headingTrajectory.next();
                currindex++;
            }
    }

    /**
     * Loop method. Call to update all values.
     */
    public void update(Pose2D pose, double secondsElapsed) {
        velocity = Vector.add(pose.getPosition(), lastPose.getPosition().scale(-1)).scale(1/secondsElapsed);
        lastPose = pose;
        updateTrajectory();
        if (!isTimeout) {
            movementvect = getMovementVector(pose);
            rotationCorrection = rotPID.get(Angle.normalize(headingTrajectory.getValue(tval, currindex).get(0)-pose.getAngle()), secondsElapsed);
            rotationCorrection += Math.signum(rotationCorrection)*kStatic;
            rotationCorrection *= -1;
        }
    }

    /*
     * Returns a pose with the desired acceleration (in your original distance unit/millis^2) and anglular acceleration.
     */
    public Pose2D get() {
        return new Pose2D(movementvect.get(0), movementvect.get(1), rotationCorrection);
    }

    /**
     * <p>
     * Returns a double[] of desired speeds for all motors, in the distance units that you used for acceleration and velocity.
     * <br>
     * <p>
     * This function must be overwritten. Here are the relevant variables you have context for:
     * <li>{@code lastPose}, containing the current pose of the robot.</li>
     * <li>{@code movementvect}, containing the desired acceleration.</li>
     * <li>{@code rotationCorrection}, containing the desired angular acceleration.</li>
     */
    public double[] getPowers() {
        throw new Error("This function must be overwritten.");
    }

     /**
     * <p>
     * Returns a Vector[] of desired vectors for all wheel pods, in the distance units that you used for acceleration and velocity.
     * <br>
     * <p>
     * This function must be overwritten. Here are the relevant variables you have context for:
     * <li>{@code lastPose}, containing the current pose of the robot.</li>
     * <li>{@code movementvect}, containing the desired acceleration.</li>
     * <li>{@code rotationCorrection}, containing the desired angular acceleration.</li>
     */
    public Vector[] getVectors() {
        throw new Error("This function must be overwritten.");
    }

    /**
     * Forcibly ends a trajectory.
     */
    public void stop() {
        isTimeout = true;
        isStopped = true;
        movementvect = new Vector(0, 0);
        rotationCorrection = 0;
    }

    /**
     * Forcibly begins a trajectory.
     */
    public void start() {
        isTimeout = false;
        isStopped = false;
    }

    /**
     * Pauses movement vector computation.
     */
    public void pause() {
        isTimeout = true;
    }

    /**
     * Resumes movement vector computation.
     */
    public void resume() {
        isTimeout = false;
    }

}
