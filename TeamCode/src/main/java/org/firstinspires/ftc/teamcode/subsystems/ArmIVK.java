package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.robotcore.util.Range;

@Config
public class ArmIVK {
    // Constants
    public static double TICKS_PER_INCH = 0;
    public static double BACKDROP_OFFSET_INCHES = 0.1;
    public static double BUCKET_OFFSET_INCHES = 3.93700787;
    public static double BUCKET_OFFSET_TO_BUCKET_RADIANS = Math.toRadians(80);
    public static double MAX_BUCKET_TILT_RADIANS = Math.toRadians(180);
    public static double BUCKET_OFFSET_TO_BUCKET_SERVO_RANGE_OFFSET_RADIANS = Math.toRadians(90);
    public static double MAX_ARM_EXTENSION_INCHES = 52.7559055;

    private static double bucketTilt = 0;
    private static int slideExtension = 0;
    private static double armAngle = 0;

    private static final double sin120 = Math.sqrt(3)/2;
    private static final double cos120 = -0.5;

    /**
     * @return Whether the given parameters are within the possible mechanical ranges.
     */
    public static boolean calcIVK(double distance, double height) {
        // (0, 0) is the base of the backdrop, (0, d) is the robot position
        Vector2d slideTopPoint = new Vector2d(height/-Math.sqrt(3), height);
        // offset off of the backdrop by using perpendicular lines
        slideTopPoint = slideTopPoint.plus(new Vector2d(Math.cos(Math.toRadians(30))*BACKDROP_OFFSET_INCHES,
                Math.sin(Math.toRadians(30))*BACKDROP_OFFSET_INCHES));
        // offset by the bucket offset
        double bucketOffsetAngle = Math.toRadians(120) - BUCKET_OFFSET_TO_BUCKET_RADIANS;
        slideTopPoint = slideTopPoint.plus(new Vector2d(Math.cos(bucketOffsetAngle)*BUCKET_OFFSET_INCHES,
                Math.sin(bucketOffsetAngle)*BUCKET_OFFSET_INCHES));
        // we now have the highest point on the slide

        Vector2d slideVector = new Vector2d(slideTopPoint.getY(), slideTopPoint.getX()-distance);

        double slideExtensionInches = slideVector.magnitude();
        int newSlideExtension = (int) (slideExtensionInches * TICKS_PER_INCH);
        // if the new slide extension is outside of the slide range return false
        if (slideExtensionInches > MAX_ARM_EXTENSION_INCHES) {
            Log.println(Log.WARN, "Arm IVK", "Exceeded maximum slide extension.");
            return false;
        }

        // 180 - bucket offset - slide angle by corresponding angles + angle of elevation
        double offsetToSlideAngle = Math.PI - bucketOffsetAngle - slideVector.angle();
        double newBucketTilt = (offsetToSlideAngle - BUCKET_OFFSET_TO_BUCKET_SERVO_RANGE_OFFSET_RADIANS)/MAX_BUCKET_TILT_RADIANS;

        // if the tilt is out of bounds return false
        if (newBucketTilt > 1) {
            Log.println(Log.WARN, "Arm IVK", "Exceeded maximum bucket tilt.");
            return false;
        } else if (newBucketTilt < 0) {
            Log.println(Log.WARN, "Arm IVK", "Exceeded minimum bucket tilt.");
            return false;
        }

        // this can't really exceed any limits unless its negative
        double newArmAngle = Math.toDegrees(slideVector.angle());
        if (newArmAngle < 0 || newArmAngle > Math.PI) {
            Log.println(Log.WARN, "Arm IVK", "Calculated arm angle exceeds arm limits.");
            return false;
        }

        // if all checks have been passed, update the values and return true
        slideExtension = newSlideExtension;
        bucketTilt = newBucketTilt;
        armAngle = newArmAngle;
        return true;

    }

    /**
     * @return In servo range (0-1).
     */
    public static double getBucketTilt() {
        return ArmIVK.bucketTilt;
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
