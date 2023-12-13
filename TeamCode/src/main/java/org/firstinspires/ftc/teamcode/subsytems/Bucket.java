package org.firstinspires.ftc.teamcode.subsytems;

import static org.firstinspires.ftc.teamcode.cachinghardwaredevice.constants.ACCEPTABLE_SERVO_POS_DELTA;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.cachinghardwaredevice.CachingServo;
import org.firstinspires.ftc.teamcode.drivers.APDS9960;

@Config
public class Bucket {
    // TODO: entire class needs implementation and testing on hardware
    private CachingServo leftLatch;
    private CachingServo rightLatch;
    private CachingServo bucketTilt;

    private APDS9960 leftSensor;
    private APDS9960 rightSensor;

    private boolean leftHasPixels = false;
    private boolean rightHasPixels = false;
    private int numPixels = 0;

    // TODO: empirically find all of these
    public static double latchPos = 0;
    public static double unlatchPos = 1;
    public static double doublePixelPos = 0.5;
    public static double intakePos = 0;
    public static double outtakePos = 1;
    public static int intakeMaxProx = 170;

    public Bucket(HardwareMap hmap) {
        leftLatch = new CachingServo(hmap.servo.get("leftLatch"));
        rightLatch = new CachingServo(hmap.servo.get("rightLatch"));
        bucketTilt = new CachingServo(hmap.servo.get("bucketTilt"));
        leftSensor = APDS9960.fromHMap(hmap, "bucketLeftSensor");
        rightSensor = APDS9960.fromHMap(hmap, "bucketRightSensor");
    }

    /**
     * Updates cached color sensor readings. Call this in the main loop.
     */
    public void update() {

        int leftprox = leftSensor.getProximity();
        int rightprox = rightSensor.getProximity();
        leftHasPixels = leftprox < intakeMaxProx;
        rightHasPixels = rightprox < intakeMaxProx;
        numPixels = (leftHasPixels?1:0) + (rightHasPixels?1:0);

    }

    /**
     * This method does not work if intaking from stacks.
     */
    public int getNumPixels() {
        return numPixels;
    }

    public void latch() {
        leftLatch.setPosition(latchPos);
        rightLatch.setPosition(latchPos);
    }

    public void unlatch() {
        leftLatch.setPosition(unlatchPos);
        rightLatch.setPosition(unlatchPos);
    }

    public boolean leftIsLatched() {
        return compare(leftLatch.getPosition(), latchPos, ACCEPTABLE_SERVO_POS_DELTA);
    }

    public boolean rightIsLatched() {
        return compare(rightLatch.getPosition(), latchPos, ACCEPTABLE_SERVO_POS_DELTA);
    }

    /**
     * Returns true if and only if both sides are latched.
     */
    public boolean isLatched() {
        return (leftIsLatched()&&rightIsLatched());
    }

    /**
     * Allows single dropping. Booleans specify which side to unlatch (true means unlatch, false means latch).
     */
    public void setlatch(boolean unlatchLeft, boolean unlatchRight) {
        if (unlatchLeft) {
            leftLatch.setPosition(unlatchPos);
        } else {
            leftLatch.setPosition(latchPos);
        }
        if (unlatchRight) {
            rightLatch.setPosition(unlatchPos);
        } else {
            rightLatch.setPosition(latchPos);
        }
    }

    /**
     * Latches only the sides that have pixels in them.
     */
    public void smartLatch() {
        if (leftHasPixels) {
            leftLatch.setPosition(latchPos);
        } else {
            leftLatch.setPosition(unlatchPos);
        }
        if (rightHasPixels) {
            rightLatch.setPosition(latchPos);
        } else {
            rightLatch.setPosition(unlatchPos);
        }
    }

    /**
     * Used if there are two pixels in one side of the bucket.
     */
    public void dropFromStack() {
        leftLatch.setPosition(doublePixelPos);
        rightLatch.setPosition(doublePixelPos);
    }

    public void intake() {
        bucketTilt.setPosition(intakePos);
        smartLatch();
    }

    private boolean compare(double v1, double v2, double tol) {
        return Math.abs(v1-v2) < tol;
    }

}
