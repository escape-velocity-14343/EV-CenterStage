package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.robotcore.util.Range;

@Config
public class ArmIVK {
    // Constants
    public static double TICKS_PER_INCH = 38;
    public static double BACKDROP_OFFSET_INCHES = 0.1;
    public static double BUCKET_OFFSET_INCHES = 3.93700787;
    public static double BUCKET_OFFSET_TO_BUCKET_RADIANS = Math.toRadians(100);
    public static double BUCKET_LENGTH_INCHES = 3.25;
    public static double MAX_BUCKET_TILT_RADIANS = 4.00553063333;
    public static double BUCKET_OFFSET_TO_BUCKET_SERVO_RANGE_OFFSET_RADIANS = -0.6;
    public static double MAX_ARM_EXTENSION_INCHES = 52.7559055;
    public static double ARM_START_OFFSET_INCHES = -5.5;
    public static double ARM_INITIAL_LENGTH = 13;
    public static double BUCKET_OFFSET_PIVOT_OFFSET_X = 0;
    public static double BUCKET_OFFSET_PIVOT_OFFSET_Y = 0;

    private static double bucketTilt = 0;
    private static int slideExtension = 0;
    private static double armAngle = 0;

    private static final double sin120 = Math.sqrt(3)/2;
    private static final double cos120 = -0.5;

    /**
     * @return Whether the given parameters are within the possible mechanical ranges.
     */
    // TODO: suspect this math is wrong because backdrop should be 60 degrees not 120 or maybe vice versa
    public static boolean calcBackdropIVK(double distance, double height) {
        // account for reversed angles
        distance = -distance;
        Vector2d backdropSpot = new Vector2d(distance, height);
        backdropSpot = backdropSpot.plus(new Vector2d(BACKDROP_OFFSET_INCHES*Math.cos(Math.toRadians(150)), BACKDROP_OFFSET_INCHES*Math.sin(Math.toRadians(150))));
        return calcIVK(backdropSpot.getX(), backdropSpot.getY(), Math.toRadians(-120));
    }

    /**
     * @param distance In Inches.
     * @param height In Inches.
     * @param bucketAngle In Radians.
     */
    public static boolean calcIVK(double distance, double height, double bucketAngle) {
        // robot is at (0, 0)
        // midpoint of the bucket is (d, h)
        Vector2d bucketPos = new Vector2d(distance, height);
        // offset to start of bucket offset
        bucketPos = bucketPos.plus(new Vector2d(BUCKET_LENGTH_INCHES*Math.cos(Math.PI+bucketAngle), BUCKET_LENGTH_INCHES*Math.sin(Math.PI+bucketAngle)));
        // offset by bucket offset angle
        bucketPos = bucketPos.plus(new Vector2d(BUCKET_OFFSET_INCHES*Math.cos(BUCKET_OFFSET_TO_BUCKET_RADIANS+bucketAngle), BUCKET_OFFSET_INCHES*Math.sin(BUCKET_OFFSET_TO_BUCKET_RADIANS+bucketAngle)));
        // we now have the slide end pos
        // offset by slide start amount
        bucketPos = bucketPos.plus(new Vector2d(ARM_START_OFFSET_INCHES, 0));
        double newSlideExtension = bucketPos.magnitude() - ARM_INITIAL_LENGTH;
        double newArmAngle = bucketPos.angle();
        double newBucketTilt = Range.clip((-newArmAngle - bucketAngle + Math.PI - BUCKET_OFFSET_TO_BUCKET_SERVO_RANGE_OFFSET_RADIANS)/MAX_BUCKET_TILT_RADIANS, 0, 1);
        // safety checks
        if (newSlideExtension > MAX_ARM_EXTENSION_INCHES) {
            Log.println(Log.WARN, "Arm IVK", "Exceeded maximum slide extension.");
            return false;
        } else if (newBucketTilt > 1) {
            Log.println(Log.WARN, "Arm IVK", "Exceeded maximum bucket tilt.");
            return false;
        } else if (newBucketTilt < 0) {
            Log.println(Log.WARN, "Arm IVK", "Exceeded minimum bucket tilt.");
            return false;
        } else if (newArmAngle < 0 || newArmAngle > Math.PI) {
            Log.println(Log.WARN, "Arm IVK", "Calculated arm angle exceeds arm limits.");
            return false;
        } else {
            slideExtension = (int) (newSlideExtension * TICKS_PER_INCH);
            armAngle = Math.toDegrees(newArmAngle);
            bucketTilt = newBucketTilt;
            return true;
        }

    }

    /**
     * @return In servo range (0-1).
     */
    public static double getBucketTilt() {
        return ArmIVK.bucketTilt;
    }

    /**
     * @param armAngle In Radians.
     * @param desiredBucketTilt In Radians.
     * @return In servo range (0-1).
     */
    public static double getBucketTilt(double armAngle, double desiredBucketTilt) {
        // sillyness to get this to function properly lmfao
        return Range.clip((-armAngle - desiredBucketTilt + Math.PI - BUCKET_OFFSET_TO_BUCKET_SERVO_RANGE_OFFSET_RADIANS)/MAX_BUCKET_TILT_RADIANS, 0, 1);

    }

    /**
     * @return In ticks.
     */
    public static int getSlideExtension() {
        return ArmIVK.slideExtension;
    }

    /**
     * @return In degrees.
     */
    public static double getArmAngle() {
        return ArmIVK.armAngle;
    }

    /**
     * DO NOT USE THIS FUNCTION UNLESS YOU ARE 100% SURE YOU KNOW WHAT YOU ARE DOING! THIS WILL BREAK THE REST OF THE IVK MATH.
     * @param ticks
     */
    public static void setSlideExtension(int ticks) {
        slideExtension = ticks;
    }
}
