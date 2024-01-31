package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.subsystems.ArmIVK.LIFTER_DOWNWARD_OFFSET;
import static org.firstinspires.ftc.teamcode.subsystems.ArmIVK.LIFTER_LENGTH;
import static org.firstinspires.ftc.teamcode.subsystems.ArmIVK.LIFTER_OFFSET_TO_ZERO;
import static org.firstinspires.ftc.teamcode.subsystems.ArmIVK.MAX_BUCKET_TILT_RADIANS;
import static org.firstinspires.ftc.teamcode.subsystems.ArmIVK.MAX_LIFTER_TILT_RADIANS;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.cachinghardwaredevice.CachingDcMotor;
import org.firstinspires.ftc.teamcode.cachinghardwaredevice.CachingMotor;
import org.firstinspires.ftc.teamcode.cachinghardwaredevice.CachingServo;
import org.firstinspires.ftc.teamcode.controllers.SquIDController;
import org.firstinspires.ftc.teamcode.controllers.SquIDF;
import org.firstinspires.ftc.teamcode.drivers.AS5600;

@Config
public class Arm {
    // TODO: entire class needs implementation and testing on hardware
    private CachingDcMotor slideMotor1;
    private CachingDcMotor slideMotor0;
    private CachingMotor tiltMotor;
    private CachingServo lifter;
    private AS5600 tiltSensor;
    private AnalogInput bucketUltrasonic;

    // TODO: empirically find these
    public static int SLIDES_MAX_EXTENSION_VALUE = 2500;
    public static double COAXIAL_EFFECT_CORRECTION = 0.504 ;
    public static double MIN_POWER = 0.2;
    public static double slidekG = 0.11;
    public static double tiltkG = 0.1;
    public static double tiltSlideGain = 0.0012;
    public static boolean useGainSchedule = false;
    private InterpLUT slidekS = new InterpLUT();
    public static double tiltP  = 0.02;
    public static double slideP = 0.003;
    public static double tiltOffset = 42;
    public static double tiltInc = 0.1;
    public static double tiltDec = 0.1;
    public static double tiltTolerance = 0.1;
    public static boolean reverseLifter = true;
    /**
     *
     * Max ticks per second when slide is stalled.
     */
    public static double STALL_VELOCITY = 0.000001;

    private SquIDF slideIQID = new SquIDF(slideP,0,0, 0, 0.1, tiltDec);
    private SquIDF tiltIQID = new SquIDF(tiltP,0,0, 0, 0.1, 0.1);

    private int slidePos = 0;
    private int coaxialCorrection = 0;
    private double tiltPos = 0;

    /**
     * Ticks per second.
     */
    private double slideVelocity = 0;
    private double tiltVelocity = 0;
    private double lastPower = 0;

    private double tiltTarget = 0;
    private double slideTarget = 0;
    private double ultrasonicInches = 0;

    private boolean hasMoved = false;
    public static double lifterOuttakePos = 0.37;
    Telemetry telemetry;



    public Arm(HardwareMap hmap, Telemetry telemetry) {

        this.slideMotor0 = new CachingDcMotor(hmap.dcMotor.get("slides0"));
        this.slideMotor1 = new CachingDcMotor(hmap.dcMotor.get("slides1"));
        this.tiltMotor = new CachingMotor(hmap, "tilt");
        this.tiltSensor = new AS5600(hmap, "tiltSensor");
        this.lifter = new CachingServo(hmap.servo.get("dummy"));
        this.bucketUltrasonic = hmap.analogInput.get("bucketUltrasonic");

        slideMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
        slideMotor0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideMotor0.setDirection(DcMotorSimple.Direction.REVERSE);
        tiltMotor.setRunMode(Motor.RunMode.RawPower);

        slidekS.add(0, 0.3);
        slidekS.add(1000, 0.1);
        slidekS.createLUT();

        setArmOffset(tiltOffset);
        setSensorInverted(true);

        this.telemetry = telemetry;

        slideIQID.setFeedforwardBounds(0, 0.2);
        tiltIQID.setFeedforwardBounds(0, 0.4);
        tiltIQID.setFeedforwardTolerance(tiltTolerance);


    }

    public void reset() {
        slideMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //slideMotor1.resetEncoder();
        //tiltMotor.resetEncoder();
    }

    public void resetSlides() {
       reset();
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
        /*if (slidePos>1700&&power>0) {
            power=0;
        }*/
        // validate bounds
        power = Range.clip(power, -1, 1);
        this.lastPower = power;
        slideMotor0.setPower(power);
        slideMotor1.setPower(power);
        this.hasMoved = true;
    }

    public void moveTilt(double power) {
        /*if (tiltPos>180&&power>0) {
            power=-power;
        }*/
        tiltMotor.set(power);
        this.hasMoved = true;
    }
    public void setLifterHeight(double height) {
        double angle = Math.acos((height+LIFTER_DOWNWARD_OFFSET)/LIFTER_LENGTH);
        if (!reverseLifter) {
            lifter.setPosition(Range.clip(angle/MAX_LIFTER_TILT_RADIANS+LIFTER_OFFSET_TO_ZERO, 0, 1));
        }
        else {
            lifter.setPosition(1-Range.clip(angle/MAX_LIFTER_TILT_RADIANS+LIFTER_OFFSET_TO_ZERO, 0, 1));
        }

    }

    /**
     * Updates the cached encooder values for the arm. Please call this every loop.
     */
    public void update(long nanos) {
        tiltIQID.setP(tiltP);
        slideIQID.setP(slideP);
        double lastPos = this.slidePos;
        double lastTilt = this.tiltPos;
        this.slidePos = slideMotor1.getCurrentPosition();// - this.coaxialCorrection;
        this.tiltPos = tiltSensor.getDegrees();
        this.coaxialCorrection = (int) (this.tiltPos * COAXIAL_EFFECT_CORRECTION);
        telemetry.addData("coax correction",coaxialCorrection);
        telemetry.addData("slide pos from arm",slidePos);
        this.slideVelocity = (this.slidePos-lastPos)/(nanos/0.000000001);
        this.tiltVelocity = (this.tiltPos-lastTilt)/(nanos/0.000000001);
        this.ultrasonicInches = (this.bucketUltrasonic.getVoltage() * 500.0 / 3.3)/2.54;
        this.hasMoved = false;

    }

    public void tiltArm(double degrees) {
        this.tiltTarget = degrees;
        double power = tiltIQID.calculate(0, AngleUnit.normalizeDegrees(degrees-tiltPos), 0.5) + this.slidePos * tiltSlideGain * Math.cos(Math.toRadians(degrees));
        if (Math.signum(power) == Math.cos(Math.toRadians(degrees))) {
            power += (tiltkG ) * Math.cos(Math.toRadians(degrees));
        }
        moveTilt(power);
        this.hasMoved = true;
    }

    /**
     * @param tolerance In degrees.
     */
    public boolean isTilted(double tolerance) {
        if (Math.abs(AngleUnit.normalizeDegrees(tiltTarget-tiltPos)) > tolerance) {
            return false;
        } else return true;
    }

    /**
     * @param tolerance In degrees.
     */
    public boolean isTilted(double tolerance, double degrees) {
        if (Math.abs(AngleUnit.normalizeDegrees(degrees-tiltPos)) > tolerance) {
            return false;
        } else return true;
    }

    /**
     * PID function for the arm slides.
     * @param target Location to PID to in ticks.
     */
    public void extend(double target) {
        //target = Range.clip(target, 0, SLIDES_MAX_EXTENSION_VALUE);
        this.slideTarget = target;
        double power = slideIQID.calculate(slidePos, target, 3);
        double ff = slidekG;
        // add kstatic friction term
        // TODO: add back the kS term
        //ff += Math.signum(power) * slidekS.get(slidePos);
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
     * PID based on ultrasonic distance. Uses extendInches() and the ArmIVK conversion value.
     * @param distance Target ultrasonic distance, in inches.
     */
    public void ultrasonicExtend(double distance) {
        double error = this.ultrasonicInches-distance;
        error *= ArmIVK.TICKS_PER_INCH;
        extend(this.slidePos+error);
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
    public void outtakeLifter() {
        lifter.setPosition(lifterOuttakePos);
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

    public double getTarget() {
        return this.slideTarget;
    }

    public double getVelocity() {return this.slideVelocity;}

    public double getTiltVelocity() {return this.tiltVelocity;}

    public double getUltrasonicInches() {
        return this.ultrasonicInches;
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
