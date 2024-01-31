package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.cachinghardwaredevice.constants.ACCEPTABLE_SERVO_POS_DELTA;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
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
    private CachingServo bucketWrist;

    private DigitalChannel leftSensor;
    private DigitalChannel rightSensor;

    private boolean leftHasPixels = false;
    private boolean rightHasPixels = false;
    private int numPixels = 0;

    // TODO: empirically find all of these
    public static double latchPosLeft = 0;
    public static double latchPosRight = 0.05;
    public static double unlatchPos = 0.8;
    public static double doublePixelPos = 0.8;
    public static double intakePos = 0.2;
    public static double outtakePos = 1;
    public static int intakeMaxProx = 240;
    public static double pixelInDelaySeconds = 0.1;
    private ElapsedTime leftPixelInTimer = new ElapsedTime();
    private ElapsedTime rightPixelInTimer = new ElapsedTime();
    int leftprox = 0, rightprox = 0;

    public boolean disableAutoLatch = true;
    public static boolean invertBar = true;
    public static boolean invertWrist = true;

    public Bucket(HardwareMap hmap) {
        leftLatch = new CachingServo(hmap.servo.get("leftLatch"));
        rightLatch = new CachingServo(hmap.servo.get("rightLatch"));
        bucketTilt = new CachingServo(hmap.servo.get("bucketTilt"));
        bucketWrist = new CachingServo(hmap.servo.get("lifter"));
        leftSensor = hmap.digitalChannel.get("RbucketBeam");
        rightSensor = hmap.digitalChannel.get("LbucketBeam");
        leftSensor.setMode(DigitalChannel.Mode.INPUT);
        rightSensor.setMode(DigitalChannel.Mode.INPUT);
        rightLatch.setDirection(Servo.Direction.REVERSE);
        bucketTilt.scaleRange(0, 0.5);
        if (invertBar)
            bucketTilt.setDirection(Servo.Direction.REVERSE);
        if (invertWrist)
            bucketWrist.setDirection(Servo.Direction.REVERSE);
    }

    /**
     * Updates cached color sensor readings. Call this in the main loop.
     */
    public void update() {

       // leftprox = leftSensor.getProximity();
       // rightprox = rightSensor.getProximity();
        leftHasPixels = !leftSensor.getState();
        rightHasPixels = !rightSensor.getState();
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
        leftLatch.setPosition(latchPosLeft);
        rightLatch.setPosition(latchPosRight);







    }

    public void unlatch() {
        leftLatch.setPosition(unlatchPos);
        rightLatch.setPosition(unlatchPos);
    }

    public boolean leftIsLatched() {
        return compare(leftLatch.getPosition(), latchPosLeft, ACCEPTABLE_SERVO_POS_DELTA);
    }

    public boolean rightIsLatched() {
        return compare(rightLatch.getPosition(), latchPosRight, ACCEPTABLE_SERVO_POS_DELTA);
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
            leftLatch.setPosition(latchPosLeft);
        }
        if (!latchRight) {
            rightLatch.setPosition(unlatchPos);
        } else {
            rightLatch.setPosition(latchPosRight);
        }
    }

    public void setRightLatch(boolean latchLeft) {
        if (!latchLeft) {
            rightLatch.setPosition(unlatchPos);
        } else {
            rightLatch.setPosition(latchPosRight);
        }
    }

    public void setLeftLatch(boolean latchRight) {
        if (!latchRight) {
            leftLatch.setPosition(unlatchPos);
        } else {
            leftLatch.setPosition(latchPosLeft);
        }
    }

    /**
     * Latches only the sides that have pixels in them.
     */
    public void smartLatch() {
        if (!disableAutoLatch) {
            if (leftHasPixels && leftPixelInTimer.seconds() > pixelInDelaySeconds) {
                leftLatch.setPosition(latchPosLeft);
            } else {
                leftLatch.setPosition(unlatchPos);
            }
            if (rightHasPixels && rightPixelInTimer.seconds() > pixelInDelaySeconds) {
                rightLatch.setPosition(latchPosRight);
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
    public double getLeftDist() {
        return leftprox;
    }
    public double getRightDist() {
        return rightprox;
    }

    public void tilt(double pos) {
        this.tilt(0.4, pos);
    }

    public void tilt(double tilt, double wrist) {
        bucketTilt.setPosition(tilt);
        bucketWrist.setPosition(wrist);
    }

    private boolean compare(double v1, double v2, double tol) {
        return Math.abs(v1-v2) < tol;
    }

}
