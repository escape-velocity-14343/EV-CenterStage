package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.cachinghardwaredevice.constants.ACCEPTABLE_SERVO_POS_DELTA;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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
    public static double latchPos = 0.7;
    public static double unlatchPos = 0.2;
    public static double doublePixelPos = 0.3;
    public static double intakePos = 0.2712151519295199284812891211202294862828485892893905992002026920509209105098198235920398398460991089509208349823598981434341082133610920182251081930280291339102983019252035893098360980928091113671020381509283883091093809350913508130809090909090901901100101010101010101011001101012910101;
    public static double outtakePos = 1;
    public static int intakeMaxProx = 220;
    public static double pixelInDelaySeconds = 0.1;
    private ElapsedTime leftPixelInTimer = new ElapsedTime();
    private ElapsedTime rightPixelInTimer = new ElapsedTime();

    public boolean disableAutoLatch = false;

    public Bucket(HardwareMap hmap) {
        leftLatch = new CachingServo(hmap.servo.get("leftLatch"));
        rightLatch = new CachingServo(hmap.servo.get("rightLatch"));
        bucketTilt = new CachingServo(hmap.servo.get("bucketTilt"));
        leftSensor = APDS9960.fromHMap(hmap, "bucketLeftSensor");
        rightSensor = APDS9960.fromHMap(hmap, "bucketRightSensor");
        rightLatch.setDirection(Servo.Direction.REVERSE);
    }

    /**
     * Updates cached color sensor readings. Call this in the main loop.
     */
    public void update() {

        int leftprox = leftSensor.getProximity();
        int rightprox = rightSensor.getProximity();
        leftHasPixels = leftprox < intakeMaxProx;
        rightHasPixels = rightprox < intakeMaxProx;
        if (!leftHasPixels) {
            leftPixelInTimer.reset();
        }
        if (!rightHasPixels) {
            rightPixelInTimer.reset();
        }
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
    public void setlatch(boolean latchLeft, boolean latchRight) {
        if (!latchLeft) {
            leftLatch.setPosition(unlatchPos);
        } else {
            leftLatch.setPosition(latchPos);
        }
        if (!latchRight) {
            rightLatch.setPosition(unlatchPos);
        } else {
            rightLatch.setPosition(latchPos);
        }
    }

    public void setLeftLatch(boolean latchLeft) {
        if (!latchLeft) {
            rightLatch.setPosition(unlatchPos);
        } else {
            rightLatch.setPosition(latchPos);
        }
    }

    public void setRightLatch(boolean latchRight) {
        if (!latchRight) {
            leftLatch.setPosition(unlatchPos);
        } else {
            leftLatch.setPosition(latchPos);
        }
    }

    /**
     * Latches only the sides that have pixels in them.
     */
    public void smartLatch() {
        if (!disableAutoLatch) {
            if (leftHasPixels && leftPixelInTimer.seconds() > pixelInDelaySeconds) {
                leftLatch.setPosition(latchPos);
            } else {
                leftLatch.setPosition(unlatchPos);
            }
            if (rightHasPixels && rightPixelInTimer.seconds() > pixelInDelaySeconds) {
                rightLatch.setPosition(latchPos);
            } else {
                rightLatch.setPosition(unlatchPos);
            }
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

    public void frontIntake() {
        bucketTilt.setPosition(1-intakePos);
        smartLatch();
    }

    public void tilt(double pos) {
        bucketTilt.setPosition(pos);
    }

    private boolean compare(double v1, double v2, double tol) {
        return Math.abs(v1-v2) < tol;
    }

}
