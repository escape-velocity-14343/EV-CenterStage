package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
public class ArmIVK {
    // Constants
    public static double TICKS_PER_INCH = 39.5;
    public static double BACKDROP_OFFSET_INCHES = 0;
    public static double BUCKET_OFFSET_INCHES = 3.93700787;
    public static double BUCKET_OFFSET_TO_BUCKET_RADIANS = Math.toRadians(100);
    public static double BUCKET_LENGTH_INCHES = 4;
    public static double MAX_BUCKET_TILT_RADIANS = 4.00553063333;
    public static double BUCKET_OFFSET_TO_BUCKET_SERVO_RANGE_OFFSET_RADIANS = -0.5;
    public static double LIFTER_LENGTH = 3.3464;

    public static double LIFTER_OFFSET_TO_ZERO = -0.095;
    public static double LIFTER_DOWNWARD_OFFSET = 0;
    public static double MAX_ARM_EXTENSION_INCHES = 52.7559055;
    public static double ARM_START_OFFSET_INCHES = -5.5;
    public static double ARM_INITIAL_LENGTH = 13;
    public static double BUCKET_OFFSET_PIVOT_OFFSET_X = 0;
    public static double BUCKET_OFFSET_PIVOT_OFFSET_Y = 0;
    public static double BUCKET_OFFSET_X = 0;
    public static double BUCKET_OFFSET_Y = 0;

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

        Vector2d bucketOff = new Vector2d(-BUCKET_OFFSET_X, -BUCKET_OFFSET_Y);
        bucketOff.rotateBy(Math.toDegrees(AngleUnit.normalizeRadians(Math.PI-bucketAngle)));
        bucketPos = bucketPos.plus(bucketOff);
        bucketPos = bucketPos.plus(new Vector2d(ARM_START_OFFSET_INCHES, 0));

        Vector2d slideOff = new Vector2d(BUCKET_OFFSET_PIVOT_OFFSET_X, BUCKET_OFFSET_PIVOT_OFFSET_Y);
        double currLen = bucketPos.magnitude();
        double slidelen = Math.sqrt(Math.pow(currLen, 2) - Math.pow(BUCKET_OFFSET_PIVOT_OFFSET_Y, 2));
        Log.println(Log.INFO, "ARMIVK", "prev angle: " + bucketPos.angle());
        Log.println(Log.INFO, "ARMIvk", "Angle correction: " + Math.atan2(BUCKET_OFFSET_PIVOT_OFFSET_X, slidelen));
        double newArmAngle = bucketPos.angle() - Math.atan2(BUCKET_OFFSET_PIVOT_OFFSET_X, slidelen);
        //bucketPos = bucketPos.plus(new Vector2d(BUCKET_OFFSET_INCHES*Math.cos(BUCKET_OFFSET_TO_BUCKET_RADIANS+bucketAngle), BUCKET_OFFSET_INCHES*Math.sin(BUCKET_OFFSET_TO_BUCKET_RADIANS+bucketAngle)));
        // we now have the slide end pos
        // offset by slide start amount

        double newSlideExtension = currLen - ARM_INITIAL_LENGTH + BUCKET_OFFSET_PIVOT_OFFSET_X;
        //double newArmAngle = bucketPos.angle();
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
     * @param height In Inches.
     * @return In Servo range (0-1).
     * */
    public static double getLifterTilt(double height) {
        double angle = Math.acos(height+LIFTER_DOWNWARD_OFFSET/LIFTER_LENGTH);
        return Range.clip(angle/MAX_BUCKET_TILT_RADIANS+LIFTER_OFFSET_TO_ZERO, 0, 1);
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
