package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.cachinghardwaredevice.CachingMotor;
import org.firstinspires.ftc.teamcode.controllers.SquIDController;
import org.firstinspires.ftc.teamcode.drivers.AS5600;

@Config
public class Arm {
    // TODO: entire class needs implementation and testing on hardware
    private CachingMotor slideMotor1;
    private CachingMotor slideMotor0;
    private CachingMotor tiltMotor;
    private AS5600 tiltSensor;

    // TODO: empirically find these
    public static int SLIDES_MAX_EXTENSION_VALUE = 2500;
    public static double COAXIAL_EFFECT_CORRECTION = 0.504;
    public static double MIN_POWER = 0.5;
    public static double slidekG = 0.1;
    public static double tiltkG = 0.1;
    public static boolean useGainSchedule = false;
    private InterpLUT slidekS = new InterpLUT();

    public static double tiltP  = 0.02;

    public static double slideP = 0.0005;
    /**
     *
     * Max ticks per second when slide is stalled.
     */
    public static double STALL_VELOCITY = 5;

    private SquIDController slideIQID = new SquIDController(slideP,0,0);
    private SquIDController tiltIQID = new SquIDController(tiltP,0,0);

    private int slidePos = 0;
    private int coaxialCorrection = 0;
    private double tiltPos = 0;

    /**
     * Ticks per second.
     */
    private double slideVelocity = 0;

    private double lastPower = 0;

    private double tiltTarget = 0;
    private double slideTarget = 0;

    private boolean hasMoved = false;



    public Arm(HardwareMap hmap) {

        this.slideMotor0 = new CachingMotor(hmap, "slides0");
        this.slideMotor1 = new CachingMotor(hmap, "slides1");
        this.tiltMotor = new CachingMotor(hmap, "tilt");
        this.tiltSensor = new AS5600(hmap, "tiltSensor");
        slideMotor1.setRunMode(Motor.RunMode.RawPower);
        slideMotor1.setInverted(true);
        slideMotor0.setInverted(true);
        slideMotor0.setRunMode(Motor.RunMode.RawPower);
        tiltMotor.setRunMode(Motor.RunMode.RawPower);
        slidekS.add(1000, 0.1);
        slidekS.add(0, 0.3);
        slidekS.createLUT();

    }

    public void reset() {
        slideMotor0.resetEncoder();
        //slideMotor1.resetEncoder();
        //tiltMotor.resetEncoder();
    }

    public void resetSlides() {
        slideMotor0.resetEncoder();
        //slideMotor1.resetEncoder();
    }

    @Deprecated
    /**
     * This function is useless because we use an absolute encoder.
     */
    public void resetTilt() {
        //tiltMotor.resetEncoder();
    }

    public void moveSlides(double power) {
        // validate bounds
        power = Range.clip(power, -1, 1);
        this.lastPower = power;
        slideMotor0.set(power);
        slideMotor1.set(power);
        this.hasMoved = true;
    }

    public void moveTilt(double power) {
        tiltMotor.set(power);
        this.hasMoved = true;
    }

    /**
     * Updates the cached encooder values for the arm. Please call this every loop.
     */
    public void update(long nanos) {
        tiltIQID.setP(tiltP);
        slideIQID.setP(slideP);
        double lastPos = slidePos;
        this.slidePos = slideMotor1.getCurrentPosition() - coaxialCorrection;
        this.tiltPos = tiltSensor.getDegrees();
        this.coaxialCorrection = (int) (this.tiltPos * COAXIAL_EFFECT_CORRECTION);
        this.slideVelocity = (slidePos-lastPos)/(nanos/0.000000001);
        this.hasMoved = false;
    }

    public void tiltArm(double degrees) {
        this.tiltTarget = degrees;
        tiltMotor.set(tiltIQID.calculate(tiltPos, degrees) + tiltkG * Math.cos(Math.toRadians(degrees)));
        this.hasMoved = true;
    }

    /**
     * @param tolerance In degrees.
     */
    public boolean isTilted(double tolerance) {
        if (Math.abs(tiltTarget-tiltPos) > tolerance) {
            return false;
        } else return true;
    }

    /**
     * @param tolerance In degrees.
     */
    public boolean isTilted(double tolerance, double degrees) {
        if (Math.abs(degrees-tiltPos) > tolerance) {
            return false;
        } else return true;
    }

    /**
     * PID function for the arm slides.
     * @param target Location to PID to in ticks.
     */
    public void extend(double target) {
        target = Range.clip(target, 0, SLIDES_MAX_EXTENSION_VALUE);
        this.slideTarget = target;
        double power = slideIQID.calculate(slidePos, target);
        double ff = slidekG;
        // add kstatic friction term
        ff += Math.signum(power) * slidekS.get(slidePos);
        ff *= Math.abs(Math.sin(Math.toRadians(tiltPos)));
        moveSlides( power + ff);
    }

    /**
     * Function for convenience. Uses arm.extend() and the ArmIVK conversion value.
     * @param target Location to PID to in inches.
     */
    public void extendInches(double target) {
        this.extend((int) (target * ArmIVK.TICKS_PER_INCH));
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

    /**
     * Hold the position of the arm. This function will automatically be overwritten by other movement calls.
     */
    public void holdPosition() {
        if (!this.hasMoved) {
            //extend(this.slidePos);
            //tiltArm(this.tiltPos);
        }
    }

    public int getPosition() {
        return this.slidePos;
    }

    /**
     * @return In degrees.
     */
    public double getTilt() {
        return this.tiltPos;
    }
    public void setArmOffset(double offset) {
        tiltSensor.setOffset(offset);
    }
    public void setSensorInverted(boolean invert) {
        tiltSensor.setInverted(invert);
    }

}
