package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.subsystems.SwerveModule.compare;


import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.drivebase.RobotDrive;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.cachinghardwaredevice.CachingMotor;
import org.firstinspires.ftc.teamcode.controllers.SquIDController;
import org.firstinspires.ftc.teamcode.drivers.AnalogEncoder;
import org.firstinspires.ftc.teamcode.drivers.ToggleTelemetry;
import org.firstinspires.ftc.teamcode.pathutils.AutonomousWaypoint;


@Config
public class SwerveController extends RobotDrive {
    public static double p = 0.01;
    public static double i = 0.0;
    public static double d = 0.005;

    public static double teleopHeadingInputGain = -0.07;

    public static double teleopHeadingFeedforwardGain = 1;

    public static double teleopHeadingChangeTolerance = 0.01;



    public static double headingwheelratio = 1.0;

    public static double rotationConstant = 0.25974026;
    public static double kTop = 1;
    public static double kBottom = -1;
    public static boolean optimize = true;
    double lt,rt;
    double jx;
    double jy;
    double rot;
    double lx,ly,rx,ry;
    double[] powers = {0,0,0,0};
    double headinglim = 1;
    double accellim = 1;

    boolean rotationInput = false;
    enum rotationMode {
        HEADING_LOCK,
        PSUEDO_LOCK,
        LOCKLESS,
        BASE,
        AUTON
    }

    rotationMode rotMode = rotationMode.PSUEDO_LOCK;

    double lastHeading = 0;
    PIDController headingPID  = new PIDController(0,0,0);
    SquIDController headingIQID = new SquIDController(0, 0, 0);
    SquIDController poscontroller = new SquIDController(Robot.kPosQ, Robot.kPosI, Robot.kPosD);

    double voltageLimit = 0;

    private Rotation2d lastMovement = new Rotation2d();


    private Robot robot;

    // stall detection things
    private boolean isMoving = false;
    private double moveMinimum = 0.3;
    // this gets reset when we stop moving (from swerve perspective)
    private ElapsedTime moveTimer = new ElapsedTime();



    Translation2d translation = new Translation2d();
    public SwerveModule left, right;
    IMU imu;
    ToggleTelemetry telemetry;
    VoltageSensor voltageSensor;
    boolean headingLock = false;
    double headingLockAngle = 0;

    public SwerveController(HardwareMap hMap, ToggleTelemetry telemetry, Robot robot) {
        this.telemetry = telemetry;
        left = new SwerveModule(new CachingMotor(hMap,"bottomleft"),new CachingMotor(hMap,"topleft"),new AnalogEncoder(hMap,"leftrot"),telemetry);

        left.setSide(false);
        right = new SwerveModule(new CachingMotor(hMap,"bottomright"),new CachingMotor(hMap,"topright"),new AnalogEncoder(hMap,"rightrot"),telemetry);

        right.setSide(true);
        left.setOffset(180); //TODO: fix swerve pod
        this.robot = robot;
        voltageSensor = robot.voltageSensor;
    }
    private void drive(double x, double y, double rot, double botHeading) {


        // normalize
        double scalar = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
        if (scalar > moveMinimum) {
            isMoving = true;
        } else {
            isMoving = false;
            moveTimer.reset();
        }
        if (scalar > 1) {
            x = x / scalar;
            y = y / scalar;
        }

        rot = Range.clip(rot, -1, 1);
        left.setPid(p,i,d);
        right.setPid(p,i,d);
        translation = new Translation2d(jx,jy);
        jx = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        jy = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        powers = new double[]{jx, jy + rot, jx, jy - rot};

        lx = left.normalize(powers, 1)[0];
        ly = left.normalize(powers, 1)[1];
        rx = left.normalize(powers, 1)[2];
        ry = left.normalize(powers, 1)[3];
        lt = Math.atan2(ly,lx);
        rt = Math.atan2(ry,rx);
        lt = Math.toDegrees(lt);
        rt = Math.toDegrees(rt);
        if (voltageSensor.getVoltage()<voltageLimit) {
            left.setMaxPower(voltageLimit-voltageSensor.getVoltage());
            right.setMaxPower(voltageLimit-voltageSensor.getVoltage());
        }
        else {
            left.setMaxPower(1);
            right.setMaxPower(1);
        }

        if (!compare(Math.abs(x)+Math.abs(y)+Math.abs(rot),0.0,0.001)) {
            lastMovement = new Rotation2d(x, y);
            left.podPidXY(lx,ly);
            right.podPidXY(rx,ry);
        }
        else {
            left.podPid(0.0, lastMovement.getDegrees());
            right.podPid(0.0, lastMovement.getDegrees());
        }
        //telemetry.addData("voltage", voltageSensor.getVoltage());
        //telemetry.addData("Left encoder", left.rot.getDegrees());
        //telemetry.addData("Right encoder", right.rot.getDegrees());
        //telemetry.addData("yaw",imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));

    }

    /**
     * For convenience.
     * @param angle In radians.
     */
    public void polarDriveFieldCentric(double angle, double power, double rot) {
        this.driveFieldCentric(Math.cos(angle)*power, Math.sin(angle)*power, rot);
    }


    public void goofyDrive(double direction, double power) {
        left.setPid(p,i,d);
        right.setPid(p,i,d);
        if (power>0.01) {
            left.podPid(power,direction);
            right.podPid(power,direction);
        }
        else {
            left.podPid(0.0, 90);
            right.podPid(0.0, 90);
        }

    }
    public void driveFieldCentric(double x, double y, double rot) {

        double botHeading = robot.getHeading();

        telemetry.addData("heading",Math.toDegrees(botHeading));
        double speed = Math.abs(x)+Math.abs(y);
        double p = Robot.kHeadingP;
        if (!compare(speed,0,0.1)) {
            p/=4;
        }
        headingIQID.setPID(p, Robot.kHeadingI, Robot.kHeadingD);
        headingIQID.setSetPoint(AngleUnit.normalizeDegrees(Math.toDegrees(headingLockAngle - botHeading)));

        switch (rotMode) {
            case HEADING_LOCK:
                headingPID.setPID(p, Robot.kHeadingI, Robot.kHeadingD);
                headingPID.setSetPoint(AngleUnit.normalizeDegrees(Math.toDegrees(headingLockAngle - botHeading)));
                telemetry.addData("set point", AngleUnit.normalizeDegrees(Math.toDegrees(headingLockAngle - botHeading)));
                if (!runIQID) {
                    rot = -headingPID.calculate(0);
                } else {
                    rot = -headingIQID.calculate(0);
                }
                telemetry.addData("rot", rot);
                break;
            case PSUEDO_LOCK:
                // state switch if joystick pressed
                if (!compare(rot, 0, 0.01)) {
                    rotMode = rotationMode.LOCKLESS;
                    break;
                }
                headingPID.setPID(p, Robot.kHeadingI, Robot.kHeadingD);
                headingPID.setSetPoint(AngleUnit.normalizeDegrees(Math.toDegrees(headingLockAngle - botHeading)));
                telemetry.addData("set point", AngleUnit.normalizeDegrees(Math.toDegrees(headingLockAngle - botHeading)));
                if (!runIQID) {
                    rot = -headingPID.calculate(0);
                } else {
                    rot = -headingIQID.calculate(0);
                }
                telemetry.addData("rot", rot);
                break;
            case LOCKLESS:
                // if no joystick input and no change in heading, state transition & set anglelock
                if (compare(rot, 0, 0.01) && compare(AngleUnit.normalizeDegrees(botHeading-lastHeading), 0, teleopHeadingChangeTolerance)) {
                    headingLockAngle = botHeading;
                    rotMode = rotationMode.PSUEDO_LOCK;
                    break;
                }
                break;
            case BASE:
                if (!compare(rot, 0, 0.01) || !compare(x, 0, 0.01) || !compare(y, 0, 0.01)) {
                    rotMode = rotationMode.LOCKLESS;
                    break;
                }
                break;
            case AUTON:
                break;
        }

        lastHeading = botHeading;

        // drive using inputs
        drive(Range.clip(x, -accellim, accellim),Range.clip(y, -accellim, accellim),Range.clip(rot, -headinglim, headinglim),botHeading);
    }
    public void driveRobotCentric(double x, double y, double rot) {
        drive(x,y,rot,0);
    }

    @Override
    public void stop() {
        headingLock(false, headingLockAngle);
        driveRobotCentric(0, 0, 0);
    }

    public boolean autonFlipHeading = false;

    public void flipHeading(boolean isFlipped) {
        autonFlipHeading = isFlipped;
    }


    /**
     * @param lock Boolean - determines whether the lock will run or not.
     * @param angle In Radians.
     */
    public void headingLock(boolean lock, double angle) {
        if (lock) {
            rotMode = rotationMode.HEADING_LOCK;
        } else {
            rotMode = rotationMode.PSUEDO_LOCK;
        }
        headingLockAngle = angle * (autonFlipHeading?-1:1);
    }

    /**
     * @param angle In Degrees.
     */
    public void toggleHeadingLock(double angle) {
        if (rotMode == rotationMode.HEADING_LOCK) {
            headingLock(false,headingLockAngle);
        }
        else
            headingLock(true,Math.toRadians(angle));
    }
    public void psuedoLock(boolean lock) {
        if (!lock) {
            headingLock(true,headingLockAngle);
        } else {
            headingLock(false,headingLockAngle);
        }
    }

    /**
     * This function will change the heading lock angle if and only if the robot is in heading lock mode.
     * @param angle In Radians.
     */
    public void updateHeadingLock(double angle) {
        if (rotMode == rotationMode.HEADING_LOCK) {
            headingLock(true, angle);
        }
    }

    public void setTurnLimit(double lim) {
        headinglim = lim;
    }
    public void setLimits(double accellimit, double turnlimit) {
        this.accellim = accellimit;
        this.headinglim = turnlimit;
    }

    public void setAuton() {
        rotMode = rotationMode.AUTON;
    }
    public void setInit() { rotMode = rotationMode.BASE; }
    public void setVoltageLimit(double voltage) {
        this.voltageLimit = voltage;
    }

    public boolean runIQID = true;
    public void setIQID(boolean runIQID) {
        this.runIQID = runIQID;
    }

    private AutonomousWaypoint targetWaypoint;
    public void driveTo(Pose2d robotPose, AutonomousWaypoint endWaypoint) {
        this.setAuton();
        targetWaypoint = endWaypoint;
        Pose2d endPose = endWaypoint.getPoint().toPose2d();
        poscontroller.setPID(Robot.kPosQ, Robot.kPosI, Robot.kPosD);
        endPose = endPose.relativeTo(robotPose);
        double mag = Math.sqrt(Math.pow(endPose.getX(), 2) + Math.pow(endPose.getY(), 2));
        double arg = Math.atan2(endPose.getY(), endPose.getX());
        mag = poscontroller.calculate(mag, 0);
        double rot = endPose.getRotation().getRadians();
        if (runIQID) {
            rot = headingIQID.calculate(rot, 0);
        } else {
            rot = headingPID.calculate(rot, 0);
        }
        this.polarDriveFieldCentric(arg, mag, rot);
    }

    public boolean atPoint(Pose2d robotpose) {
        return targetWaypoint.isAtPoint(robotpose);
    }

    /**
     * Caution when using this function! swerve.atPoint() will not work.
     */
    @Deprecated
    public void driveTo(Pose2d robotPose, Pose2d endPose) {
        poscontroller.setPID(Robot.kPosQ, Robot.kPosI, Robot.kPosD);
        endPose = endPose.relativeTo(robotPose);
        double mag = Math.sqrt(Math.pow(endPose.getX(), 2) + Math.pow(endPose.getY(), 2));
        double arg = Math.atan2(endPose.getY(), endPose.getX());
        mag = poscontroller.calculate(mag, 0);
        double rot = endPose.getRotation().getRadians();
        if (runIQID) {
            rot = headingIQID.calculate(rot, 0);
        } else {
            rot = headingPID.calculate(rot, 0);
        }
        this.polarDriveFieldCentric(arg, mag, rot);
    }

    private long MINIMUM_START_MOVEMENT_NANOS = 100 * 1000000; // ms * conversion (1e6)

    /**
     * Sets the amount of time it takes until swerve starts moving, after a drive command has been given.
     */
    public void setMinMoveSeconds(double seconds) {
        MINIMUM_START_MOVEMENT_NANOS = (long) (seconds * 1e9);
    }

    public boolean isMoving() {
        return (this.isMoving && moveTimer.nanoseconds() > MINIMUM_START_MOVEMENT_NANOS);
    }
}
