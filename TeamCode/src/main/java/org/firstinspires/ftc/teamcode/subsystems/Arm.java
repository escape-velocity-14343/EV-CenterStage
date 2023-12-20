package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.cachinghardwaredevice.CachingMotor;
import org.firstinspires.ftc.teamcode.controllers.SquIDController;

@Config
public class Arm {
    // TODO: entire class needs implementation and testing on hardware
    private CachingMotor slideMotor1;
    private CachingMotor slideMotor0;
    private CachingMotor tiltMotor;

    // TODO: empirically find these
    public static double TILT_TICKS_PER_DEGREE = 0;
    public static int SLIDES_MAX_EXTENSION_VALUE = 2500;
    public static double MIN_POWER = 0.5;
    public static double slidekG = 0.1;
    public static double tiltkG = 0.1;
    /**
     * Max ticks per second when slide is stalled.
     */
    public static double STALL_VELOCITY = 5;

    private SquIDController slideIQID;
    private SquIDController tiltIQID;

    private int slidePos = 0;
    private int tiltPos = 0;

    /**
     * Ticks per second.
     */
    private double slideVelocity = 0;

    private double lastPower = 0;

    private int tiltTarget = 0;
    private double slideTarget = 0;



    public Arm(HardwareMap hmap) {

        this.slideMotor0 = new CachingMotor(hmap, "slides0");
        this.slideMotor1 = new CachingMotor(hmap, "slides1");
        this.tiltMotor = new CachingMotor(hmap, "tilt");
        slideMotor1.setRunMode(Motor.RunMode.RawPower);
        slideMotor0.setRunMode(Motor.RunMode.RawPower);
        tiltMotor.setRunMode(Motor.RunMode.RawPower);

    }

    public void reset() {
        slideMotor0.resetEncoder();
        slideMotor1.resetEncoder();
        tiltMotor.resetEncoder();
    }

    public void resetSlides() {
        slideMotor0.resetEncoder();
        slideMotor1.resetEncoder();
    }

    public void resetTilt() {
        tiltMotor.resetEncoder();
    }

    public void moveSlides(double power) {
        // validate bounds
        power = Range.clip(power, -1, 1);
        this.lastPower = power;
        slideMotor0.set(power);
        slideMotor1.set(power);
    }

    public void moveTilt(double power) {
        tiltMotor.set(power);
    }

    /**
     * Updates the cached encooder values for the arm. Please call this every loop.
     */
    public void update(long nanos) {
        double lastPos = slidePos;
        this.slidePos = slideMotor0.getCurrentPosition();
        this.tiltPos = tiltMotor.getCurrentPosition();
        this.slideVelocity = (slidePos-lastPos)/(nanos/0.000000001);
    }

    public void tiltArm(double degrees) {
        this.tiltTarget = (int) (degrees*TILT_TICKS_PER_DEGREE);
        tiltMotor.set(tiltIQID.calculate(tiltPos, degrees*TILT_TICKS_PER_DEGREE) + tiltkG * Math.abs(Math.cos(degrees)));
    }

    /**
     * @param tolerance In degrees.
     */
    public boolean isTilted(double tolerance) {
        if (Math.abs(tiltTarget-tiltPos) > tolerance*TILT_TICKS_PER_DEGREE) {
            return false;
        } else return true;
    }

    /**
     * @param tolerance In degrees.
     */
    public boolean isTilted(double tolerance, double degrees) {
        if (Math.abs(degrees-tiltPos) > tolerance*TILT_TICKS_PER_DEGREE) {
            return false;
        } else return true;
    }

    /**
     * PID function for the arm slides.
     * @param target Location to PID to in ticks.
     */
    public void extend(double target) {
        this.slideTarget = target;
        moveSlides(slideIQID.calculate(slidePos, target) + slidekG * Math.abs(Math.sin(tiltPos / TILT_TICKS_PER_DEGREE)));
    }

    /**
     * @param tolerance In ticks.
     * @return If the arm is at the desired extension.
     */
    public boolean isDone(double tolerance) {
        if (Math.abs(this.slideTarget-this.slidePos) > tolerance) {
            return false;
        } else return true;
    }


    // TODO: TUNE THESE CONSTANTS!!!
    /**
     * Stall detection! Constants need to be tuned!!!
     * @return
     */
    public boolean isStalled() {
        if (Math.abs(lastPower) > MIN_POWER && Math.abs(slideVelocity) < STALL_VELOCITY) {
            return true;
        } else {
            return false;
        }
    }

    public void holdPosition() {
        moveSlides(slidekG * Math.abs(Math.sin(tiltPos / TILT_TICKS_PER_DEGREE)));
        tiltArm(tiltkG * Math.abs(Math.cos(tiltPos / TILT_TICKS_PER_DEGREE)));
    }

    public int getPosition() {
        return this.slidePos;
    }

    /**
     * @return In degrees.
     */
    public double getTilt() {
        return this.tiltPos / TILT_TICKS_PER_DEGREE;
    }

    public int getTiltTicks() {
        return this.tiltPos;
    }

    public int getTiltTargetTicks() {
        return this.tiltTarget;
    }

}
