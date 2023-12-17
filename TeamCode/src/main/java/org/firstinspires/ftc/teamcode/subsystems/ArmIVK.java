package org.firstinspires.ftc.teamcode.subsystems;

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

    private static double bucketTilt = 0;
    private static int slideExtension = 0;
    private static double armAngle = 0;

    private static final double sin120 = Math.sqrt(3)/2;
    private static final double cos120 = -0.5;

    public static void calcIVK(double distance, double height) {
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
        slideExtension = (int) (slideExtensionInches * TICKS_PER_INCH);

        // 180 - bucket offset - slide angle by corresponding angles + angle of elevation
        double offsetToSlideAngle = Math.PI - bucketOffsetAngle - slideVector.angle();
        // TODO: fix this step to account for the bucket tilt offset
        bucketTilt = Range.clip(offsetToSlideAngle/MAX_BUCKET_TILT_RADIANS, 0, 1);
        armAngle = Math.toDegrees(slideVector.angle());


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
}
