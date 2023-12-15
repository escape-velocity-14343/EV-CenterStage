package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.cachinghardwaredevice.CachingMotor;
import org.firstinspires.ftc.teamcode.controllers.IQIDController;

@Config
public class Arm {
    // TODO: entire class needs implementation and testing on hardware
    private CachingMotor slideMotor;
    private CachingMotor tiltMotor;

    // TODO: empirically find these
    public static double TILT_TICKS_PER_DEGREE = 0;
    public static int SLIDES_MAX_EXTENSION_VALUE = 2500;
    public static double MIN_POWER = 0.5;
    /**
     * Max ticks per second when slide is stalled.
     */
    public static double STALL_VELOCITY = 5;

    private IQIDController slideIQID;
    private IQIDController tiltIQID;

    private int slidePos = 0;
    private int tiltPos = 0;

    /**
     * Ticks per second.
     */
    private double slideVelocity = 0;

    private double lastPower = 0;

    private double tiltTarget = 0;
    private double slideTarget = 0;



    public Arm(HardwareMap hmap) {

        this.slideMotor = new CachingMotor(hmap, "slides0");
        this.tiltMotor = new CachingMotor(hmap, "tilt");
        slideMotor.setRunMode(Motor.RunMode.RawPower);
        tiltMotor.setRunMode(Motor.RunMode.RawPower);

    }

    public void reset() {
        slideMotor.resetEncoder();
        tiltMotor.resetEncoder();
    }

    public void moveSlides(double power) {
        this.lastPower = power;
        slideMotor.set(power);
    }

    /**
     * Updates the cached encooder values for the arm. Please call this every loop.
     */
    public void update(long nanos) {
        double lastPos = slidePos;
        this.slidePos = slideMotor.getCurrentPosition();
        this.tiltPos = tiltMotor.getCurrentPosition();
        this.slideVelocity = (slidePos-lastPos)/(nanos/0.000000001);
    }

    public void tiltArm(double degrees) {
        this.tiltTarget = degrees*TILT_TICKS_PER_DEGREE;
        tiltMotor.set(tiltIQID.calculate(tiltPos, degrees*TILT_TICKS_PER_DEGREE));
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
        moveSlides(slideIQID.calculate(slidePos, target));
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

    public int getPosition() {
        return this.slidePos;
    }

    public double getTilt() {
        return this.tiltPos / TILT_TICKS_PER_DEGREE;
    }

}
