package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.cachinghardwaredevice.CachingDcMotor;
import org.firstinspires.ftc.teamcode.cachinghardwaredevice.CachingServo;
import org.firstinspires.ftc.teamcode.controllers.SquIDController;
import org.firstinspires.ftc.teamcode.drivers.DashboardPacketTelemetry;
import org.firstinspires.ftc.teamcode.drivers.ToggleTelemetry;
import org.firstinspires.ftc.teamcode.pathutils.AutonomousWaypoint;
import org.firstinspires.ftc.teamcode.pathutils.Point;

import java.util.List;

@Disabled
@Config
public abstract class Robot extends LinearOpMode {

    // TODO: swerve and odometry implementations
    // TODO: PID to point implementation

    /**
     * Subsystems.
     */
    public SwerveController swerve;
    public Arm arm;
    public Bucket bucket;
    public Odometry odometry;
    //public IMU imu;
    public List<LynxModule> allHubs;
    public VoltageSensor voltageSensor;

    public enum states {
        INTAKE,
        OUTTAKE,
        AUTO,
        FOLDED,
        IFOLD,
        NONE,
        INIT,
        HANG,
        DRONE

    }

    private enum intakeStates {
        EXTENDED,
        RETRACTED,
        TILTED,
        DONE
    }

    private enum outtakeStates {
        EXTENDED,
        RETRACTED,
        TILTED
    }
    private enum hangStates {
        GROUND,
        UP
    }

    private states transferStates = states.NONE;
    private intakeStates intakeProgress = intakeStates.EXTENDED;
    private outtakeStates outtakeProgress = outtakeStates.EXTENDED;
    private hangStates hangProgress = hangStates.GROUND;
    private boolean fsmIsDone = false;

    /**
     * In radians.
     */
    private double botHeading = 0;
    private double imuReading = 0;
    private double headingOffset = 0;

    /**
     * Timing.
     */
    public ElapsedTime timer = new ElapsedTime();
    private long lastLoopNanos = timer.nanoseconds();
    public long loopNanos = 0;

    /**
     * PID Config.
     */
    public static double kHeadingP = 0.1;
    public static double kHeadingI = 0;
    public static double kHeadingD = 0;
    public static double kHeadingF = 0.05;
    public static double headingInc = 0.05;
    public static double headingDec = 1;
    public static double headingTol = 0.1;

    public static double kPosQ = 0.02;
    public static double kPosI = 0;
    public static double kPosD = 0;
    private SquIDController poscontroller = new SquIDController(kPosQ, kPosI, kPosD);

    /**
     * Telemetry.
     */
    ToggleTelemetry toggleableTelemetry;
    public static boolean useDashTelemetry = false;

    /**
     * Control Config.
     */
    public Gamepad gamepad1c = new Gamepad();
    public Gamepad gamepad2c = new Gamepad();

    /**
     * FSM values.
     */
    private double lastArmIVKHeight = 0;
    private double lastArmIVKDistance = -0;
    private double intakeTilt = 0;
    private boolean disableAutoRetract = true;
    private int flipHeadingLock = 1;
    public static double MAX_TILT_CHANGE = 30;
    public static double intakeLifterHeight = 0;
    public DcMotor hang;
    public Servo drone;
    public static double droneTilt = 135;





    public void initialize() {
        hang = new CachingDcMotor(hardwareMap.dcMotor.get("intake"));
        drone = new CachingServo(hardwareMap.servo.get("drone"));
        allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        //DashboardPacketTelemetry fieldtelem = new DashboardPacketTelemetry();
       // telemetry = new MultipleTelemetry(fieldtelem, telemetry);
        //telemetry = FtcDashboard.getInstance().getTelemetry();
        if (useDashTelemetry) {
            telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
        }
        toggleableTelemetry = new ToggleTelemetry(telemetry);
        arm = new Arm(hardwareMap,telemetry);
        bucket = new Bucket(hardwareMap);
        swerve = new SwerveController(hardwareMap, toggleableTelemetry, this);
        odometry = new Odometry(hardwareMap, toggleableTelemetry);

        swerve.setInit();
        transferStates = states.INIT;

        calcArmIVK(10, 0);

        //imu = hardwareMap.get(IMU.class, "imu 1");
        //IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
        //imu.initialize(parameters);

        initialNanos = timer.nanoseconds();


    }

    @Deprecated
    public void readIMU() {
    //    imuReading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    public void setPoseEstimate(Pose2d pose) {
        if (opModeInInit()) {
            readIMU();
        }
        //Log.println(Log.INFO, "heading", "imu reading: " + imuReading);
        //headingOffset = pose.getRotation().getRadians()-imuReading;
        odometry.reset(pose.getX(), pose.getY(), pose.getRotation().getRadians());

    }

    /**
     * USE THIS FUNCTION WHENEVER YOU WANT TO GET A BOT HEADING OR USE BOTHEADING. DO NOT USE THE IMU.
     */
    public double getHeading() {

        //return botHeading = AngleUnit.normalizeRadians(imuReading + headingOffset);
        return botHeading = odometry.getPose().getRotation().getRadians();
    }

    private int loops = 0;
    private long initialNanos = 0;

    public static double bucketTiltThing = 1;
    public static double notBucketTiltThing = 0.6;
    public static boolean reverseBucketTiltThing = false;

    public void update() {
        loops++;
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }
        loopNanos = timer.nanoseconds() - lastLoopNanos;
        lastLoopNanos = timer.nanoseconds();
        telemetry.addData("actual looptime", loopNanos/1e6);
        telemetry.addData("average looptimes", (lastLoopNanos-initialNanos)/(loops*1e6));

        gamepad1c.copy(gamepad1);
        gamepad2c.copy(gamepad2);
        //imuReading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        telemetry.addData("imu looptime", (timer.nanoseconds() - lastLoopNanos)/1e6);
        telemetry.addData("imu reading", imuReading);
        telemetry.addData("heading offset", headingOffset);
        //telemetry.addData("voltage", voltageSensor.getVoltage());



        long botheadingread = timer.nanoseconds();
        odometry.updateArc(loopNanos);
        botHeading = getHeading();
        telemetry.addData("odometry looptime", (timer.nanoseconds() - botheadingread)/1e6);

        long miscread = timer.nanoseconds();
        arm.update(loopNanos);
        bucket.update();
        telemetry.addData("slide pos", arm.getPosition());
        telemetry.addData("arm velo", arm.getVelocity());
        telemetry.addData("misc looptime", (timer.nanoseconds() - miscread)/1e6);

        long fsmread = timer.nanoseconds();


        fsmIsDone = false;

        switch (transferStates) {
            case INTAKE:
                switch (intakeProgress) {
                    case EXTENDED:
                        bucket.unlatch();
                        // if arm is already in right position don't do anything
                        if (arm.isTilted(1, intakeTilt)) {
                            intakeProgress = intakeStates.TILTED;
                        } else if (arm.getPosition() < 300) {
                            intakeProgress = intakeStates.RETRACTED;
                        } else {
                            arm.moveSlides(-1);
                        }
                        break;
                    case RETRACTED:
                        bucket.tilt(1, 1);
                        ArmIVK.calcIntakeIVK(5,0, Math.toRadians(arm.getTilt()));
                        fsmIsDone = true;
                        arm.tiltArm(-2);
                        if (arm.getTilt() < 5) {
                            intakeProgress = intakeStates.TILTED;
                        }
                        break;
                    case TILTED:
                        arm.tiltArm(-2);
                        bucket.tilt(1, 1);
                        //bucket.tilt(ArmIVK.getBarTilt(), ArmIVK.getBucketTilt());
                        // tilt if the arm is in a reasonable position to tilt
                        if (arm.getPosition() < 500 || Math.abs(arm.getTilt()-intakeTilt) < 10) {
                           // arm.tiltArm(intakeTilt);
                        }
                        //bucket.tilt(bucketTiltThing, notBucketTiltThing);
                        bucket.smartLatch();
                        arm.extend(50);
                        if (arm.isDone(20)) {
                            arm.moveTilt(0);
                            intakeProgress = intakeStates.DONE;
                        }
                        break;
                    case DONE:
                        if (arm.getPosition() < 500 || Math.abs(arm.getTilt()-intakeTilt) < 10) {
                            //arm.tiltArm(intakeTilt);
                        }
                        //arm.outtakeLifter();
                        arm.setLifterHeight(intakeLifterHeight);

                        // tilt if the arm is in a reasonable position to tilt
                        bucket.tilt(ArmIVK.getBarTilt(), ArmIVK.getBucketTilt());
                        bucket.smartLatch();
                        if (bucket.getNumPixels()==2) {
                            gamepad1.rumble(50);

                        }
                        fsmIsDone = true;
                        break;
                }
                break;

            case OUTTAKE:
                switch (outtakeProgress) {
                    case EXTENDED:
                        bucket.latch();
                        // go to last known arm params
                        // if it doesn't work set to defaults
                        if (!calcArmIVK(getArmDistance(), getArmHeight())) {
                            calcArmIVK(10, 0);
                        }

                        // if arm is already in right position don't do anything
                        if (arm.isTilted(1, ArmIVK.getArmAngle())) {
                            swerve.headingLock(true, Math.PI/2 * flipHeadingLock);
                            outtakeProgress = outtakeStates.TILTED;
                        } else if (arm.getPosition() < 300) {
                            swerve.headingLock(true, Math.PI/2 * flipHeadingLock);
                            outtakeProgress = outtakeStates.RETRACTED;
                        } else {
                            arm.moveSlides(-1);
                        }

                        break;
                    case RETRACTED:
                        fsmIsDone = true;
                        safeGoToArmIVK();
                        if (arm.isTilted(3)) {
                            outtakeProgress = outtakeStates.TILTED;
                        }
                        break;
                    case TILTED:
                        goToArmIVK();
                        fsmIsDone = true;
                        break;
                }
                break;
            case INIT:
                bucket.tilt(1, 0);
                arm.setLifterHeight(0);
                drone.setPosition(1);
                break;
            case FOLDED:
                bucket.latch();
                bucket.tilt(1, 1);
                arm.setLifterHeight(0);
                arm.extend(20);
                if (arm.getPosition() < 100) {
                    arm.tiltArm(168);
                } else {
                    arm.moveTilt(0);
                }
                if (arm.isDone(20) && arm.isTilted(3)) {
                    fsmIsDone = true;
                }
                break;
            case IFOLD:
                bucket.latch();
                bucket.tilt(0.6);
                arm.extend(20);
                if (arm.isDone(10)) {
                    arm.tiltArm(10);
                }
                if (arm.isDone(10) && arm.isTilted(1)) {
                    arm.moveTilt(0);
                    arm.extend(10);
                    fsmIsDone = true;
                }
                break;
            case HANG:
                switch (hangProgress) {
                    case GROUND:
                        if (arm.getPosition() < 100) {
                            arm.tiltArm(110);
                        } else {
                            arm.moveSlides(-0.7);
                        }
                        bucket.tilt(0, 0);
                        swerve.setAuton();
                        break;
                    case UP:
                        arm.tiltArm(160);
                        //arm.extend(0);
                        bucket.tilt(0);
                        swerve.setAuton();
                }
                break;
            case DRONE:
                bucket.tilt(0, 1);
                if (arm.getPosition() < 100) {
                    arm.tiltArm(droneTilt);
                } else {
                    arm.moveSlides(-0.7);
                }


        }
        if (!inHang()) {
            swerve.setNormal();
        }
        // if nothing else is happening, hold position
        //arm.holdPosition();
        telemetry.addData("fsm looptimes", (timer.nanoseconds() - fsmread)/1e6);

        telemetry.update();
    }

    public states getState() { return transferStates; }

    public boolean armFSMIsDone() { return fsmIsDone; }

    public void intake() {
        this.transferStates = states.INTAKE;
        this.intakeProgress = intakeStates.EXTENDED;
    }

    public void setIntake(double tilt) {
        this.intakeTilt = tilt;
    }
    public double getIntakeTilt() {
        return this.intakeTilt;
    }

    public void setAutoLatch(boolean active) {
        active = !active;
        this.disableAutoRetract = active;
        this.bucket.disableAutoLatch = active;
    }

    public void setAutoRetract(boolean active) {
        active = !active;
        this.disableAutoRetract = active;
    }

    public void outtake() {
        this.transferStates = states.OUTTAKE;
        this.outtakeProgress = outtakeStates.EXTENDED;
    }

    public void setOuttake(double distance, double height) {
        calcArmIVK(distance, height);
    }

    public void foldArm() {
        this.transferStates = states.FOLDED;
    }
    public void iFoldArm() {
        this.transferStates = states.IFOLD;
    }
    public void hang() {
        this.transferStates = states.HANG;
        this.hangProgress = hangStates.GROUND;
    }

    public void drone() {
        this.transferStates = states.DRONE;
    }

    /**
     * we are flying
     */
    public void incrementHangState() {
        if (inHang()) {
            this.hangProgress = hangStates.UP;
        }
    }

    /**
     * Allows for manual control of the arm in the autonomous.
     */
    public void setFSMtoAuto() {
        this.transferStates = states.AUTO;
    }

    public boolean inOuttake() {
        return transferStates == states.OUTTAKE;
    }

    public boolean inIntake() {
        return  transferStates == states.INTAKE;
    }

    public boolean inAuto() { return transferStates == states.AUTO; }
    public boolean inHang() {
        return transferStates == states.HANG;
    }

    public boolean inDrone() {
        return transferStates == states.DRONE;
    }

    public boolean isDone() {
        return fsmIsDone;
    }

    public void flipHeadingLock() {
        this.flipHeadingLock = -flipHeadingLock;
        if (swerve.headingLock) {
            swerve.headingLock(true, -swerve.headingLockAngle);
        }
    }

    /**
     * @param height From the lowest possible pixel slot, measured in pixel slots.
     */
    public boolean calcArmIVK(double distance, double height) {
        height = Range.clip(height, 0, 50);
        // don't allow scoring from more than 2.5 tiles away to prevent minors
        distance = Range.clip(distance, 0, 60);
        lastArmIVKHeight = height;
        lastArmIVKDistance = distance;
        return ArmIVK.calcBackdropIVK(distance, height+6);
    }

    public double getArmHeight() {
        return lastArmIVKHeight;
    }

    public double getArmDistance() {
        return lastArmIVKDistance;
    }

    public void goToArmIVK() {
        if (AngleUnit.normalizeDegrees(Math.abs(arm.getTilt() - ArmIVK.getArmAngle())) > MAX_TILT_CHANGE) {
            safeGoToArmIVK();
        } else {
            arm.extend(ArmIVK.getSlideExtension());
            arm.tiltArm(ArmIVK.getArmAngle());
            bucket.tilt(1, ArmIVK.getBucketTilt());
        }
    }

    enum sillyivk {
        CLOSE,
        AT
    }

    sillyivk silly = sillyivk.CLOSE;

    /**
     * WARNING: ONLY USE THIS FUNCTION WHEN GOING TO A INTAKE POSITION!!!
     * @param tiltSlowdown
     */
    public void goToArmIVK(double tiltSlowdown) {
        if (AngleUnit.normalizeDegrees(Math.abs(arm.getTilt() - ArmIVK.getArmAngle())) > MAX_TILT_CHANGE) {
            safeGoToArmIVK();
        } else {
            arm.extend(ArmIVK.getSlideExtension());
            switch (silly) {
                case CLOSE:
                    arm.tiltArm(ArmIVK.getArmAngle() + Math.signum(arm.getTilt() - ArmIVK.getArmAngle()) * tiltSlowdown);
                    if (arm.isTilted(4) && arm.getTiltVelocity() < 1) {
                        silly = sillyivk.AT;
                    }
                    break;
                case AT:
                    arm.tiltArm(ArmIVK.getArmAngle());
                    if (AngleUnit.normalizeDegrees((arm.getTilt() - ArmIVK.getArmAngle())) > tiltSlowdown) {
                        silly = sillyivk.CLOSE;
                    }
                    break;
            }
            bucket.tilt(0, ArmIVK.getBucketTilt());
        }
    }

    enum safeGoToArmIVKFSM {
        EXTENDED,
        RETRACTED,
        TILTED
    }

    safeGoToArmIVKFSM safeFSM = safeGoToArmIVKFSM.EXTENDED;

    public void resetSafeGoToIVK() {
        safeFSM = safeGoToArmIVKFSM.EXTENDED;
    }

    public void safeGoToArmIVK() {
        if (arm.getPosition() > 100 && Math.abs(ArmIVK.getArmAngle() - arm.getTilt()) > 10) {
            safeFSM = safeGoToArmIVKFSM.EXTENDED;
        }
        switch (safeFSM) {
            case EXTENDED:
                arm.moveSlides(-0.7);
                arm.moveTilt(0);
                //bucket.tilt(ArmIVK.getBucketTilt(Math.toRadians(arm.getTilt()), Math.PI));
                if (arm.getPosition() < 100) {
                    safeFSM = safeGoToArmIVKFSM.RETRACTED;
                }
                break;
            case RETRACTED:
                arm.moveSlides(0);
                arm.tiltArm(ArmIVK.getArmAngle());
                //bucket.tilt(ArmIVK.getBucketTilt(Math.toRadians(arm.getTilt()), Math.PI));
                if (arm.isTilted(1) && arm.getTiltVelocity() < 0.2) {
                    safeFSM = safeGoToArmIVKFSM.TILTED;
                }
                break;
            case TILTED:
                arm.tiltArm(ArmIVK.getArmAngle());
                arm.extend(ArmIVK.getSlideExtension());
                bucket.tilt(1, ArmIVK.getBucketTilt());
                break;
        }
        /*if (arm.getPosition() > 100 && !arm.isTilted(10)) {

        } else if (!arm.isTilted(1) || arm.getTiltVelocity() > 0.5) {
            arm.moveSlides(0);
            arm.tiltArm(ArmIVK.getArmAngle());
            bucket.tilt(ArmIVK.getBucketTilt(Math.toRadians(arm.getTilt()), Math.PI));
        } else {
            arm.tiltArm(ArmIVK.getArmAngle());
            arm.extend(ArmIVK.getSlideExtension());
            bucket.tilt(ArmIVK.getBucketTilt());
        }*/
    }

    public void forceGoToArmIVK() {
        arm.tiltArm(ArmIVK.getArmAngle());
        arm.extend(ArmIVK.getSlideExtension());
        bucket.tilt(ArmIVK.getBucketTilt());
    }

    /*
     * PID functions!
     */

    @Deprecated
    public void goToPoint(double x, double y, double rot) {
        swerve.driveTo(odometry.getPose(), odometry.getVelocityPose(), new Pose2d(x, y, new Rotation2d(rot)));
    }

    @Deprecated
    public void goToPoint(double x, double y) {
        this.goToPoint(x, y, 0);
    }

    @Deprecated
    public void goToPoint(Pose2d endPose) {
        this.goToPoint(endPose.getX(), endPose.getY(), endPose.getRotation().getRadians());
    }

    @Deprecated
    public void goToPoint(Point endPoint) {
        this.goToPoint(endPoint.toPose2d());
    }

    @Deprecated
    public void fromPoseGoToPoint(Pose2d startpose, double x, double y, double rot) {
        this.goToPoint(fromPoseGetPoint(startpose, x, y, rot));
    }

    @Deprecated
    public void fromPoseGoToPoint(Pose2d startpose, Point point) {
        this.goToPoint(fromPoseGetPoint(startpose, point.x, point.y, point.heading));
    }

    @Deprecated
    public void fromCurrPosGoToPoint(double x, double y, double rot) {
        this.goToPoint(fromCurrPoseGetPoint(x, y, rot));
    }

    @Deprecated
    public void fromCurrPosGoToPoint(Point point) {
        this.goToPoint(fromCurrPoseGetPoint(point.x, point.y, point.heading));
    }

    public Pose2d fromPoseGetPoint(Pose2d startpose, double x, double y, double rot) {
        return new Pose2d(startpose.getX()+x, startpose.getY()+y, new Rotation2d(startpose.getRotation().getRadians()+rot));
    }

    public Pose2d fromCurrPoseGetPoint(double x, double y, double rot) {
        return this.fromPoseGetPoint(odometry.getPose(), x, y, rot);
    }

    /*
     * New functions using AutonomousWaypoint.
     */

    public void goToPoint(AutonomousWaypoint endwaypoint) {
        swerve.driveTo(odometry.getPose(), odometry.getVelocityPose(), endwaypoint, odometry);
        odometry.setTarget(endwaypoint.getPoint(odometry.getPose()).x, endwaypoint.getPoint(odometry.getPose()).y);
    }

    public boolean atPoint() {
        return swerve.atPoint(odometry.getPose());
    }

    /**
     * Stall detection using motor powers vs movement.
     * Swerve class internally corrects for initial acceleration so this should always be accurate.
     */
    public boolean swerveIsStalled() {
        return (swerve.isMoving() && !odometry.isMoving());
    }
    public static double stoppedAngularVelo = 0.0001;
    public boolean isStoppedAtPoint() {
        return atPoint()&&odometry.getAnglularVelocity()<stoppedAngularVelo&&!odometry.isMoving();
    }
}

// old robot code below
/*package org.firstinspires.ftc.teamcode.subsystems;

        import android.util.Log;

        import com.acmerobotics.dashboard.FtcDashboard;
        import com.acmerobotics.dashboard.canvas.Canvas;
        import com.acmerobotics.dashboard.config.Config;
        import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
        import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
        import com.arcrobotics.ftclib.controller.PIDController;
        import com.arcrobotics.ftclib.geometry.Pose2d;
        import com.arcrobotics.ftclib.geometry.Rotation2d;
        import com.arcrobotics.ftclib.hardware.motors.Motor;
        import com.arcrobotics.ftclib.hardware.motors.MotorEx;
        import com.qualcomm.hardware.lynx.LynxModule;
        import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.qualcomm.robotcore.hardware.Gamepad;
        import com.qualcomm.robotcore.hardware.IMU;
        import com.qualcomm.robotcore.hardware.Servo;
        import com.qualcomm.robotcore.hardware.VoltageSensor;
        import com.qualcomm.robotcore.util.ElapsedTime;

        import org.firstinspires.ftc.robotcore.external.Telemetry;
        import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
        import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
        import org.firstinspires.ftc.teamcode.drivers.AnalogEncoder;
        import org.firstinspires.ftc.teamcode.drivers.ToggleTelemetry;
        import org.firstinspires.ftc.teamcode.drivers.UltrasonicSensor;

        import java.util.List;
/*@Config
public abstract class Robot extends LinearOpMode {
    //Drive
    public SwerveController swerve;

    //Subsystems
    public Bucket bucket;
    public Intake intake;
    public IMU imu;
    public Slides slides;
    public List<LynxModule> allHubs;

    public Servo drone;
    public VoltageSensor voltageSensor;

    public static double kHeadingP = 0.01;
    public static double kHeadingI = 0;
    public static double kHeadingD = 0;

    public static double kSlidesP = 0.007;
    public static double kSlidesI = 0;
    public static double kSlidesD = 0;
    public static double kIntakeP = 0.015;
    public static double kIntakeI = 0.05;
    public static double kIntakeD = 0.0001;

    public static double kSlidesStatic = 0;

    public static int slidesPos = 0;
    public static double intakeTilt = 0.07;
    public static double outtakeTilt = 0.65;

    public static double underpassBucketPos = 1;
    public static double underpassDoneBucketPos = 0.65;

    public static int slideIntakeEncVal = 300;

    public static int slideIntakeEncTol = 70;
    public static int underpassSlidePos = 0;
    public static int bucketIntakeWaitms = 30;
    public static int slideOuttakeEncPos = 1300;
    public static int slideExtendoEncPos = 2400;
    public static int slideHangRaiseEncPos = 1800;
    public static int slideHangLowerEncPos = 400;
    public static double pixelInMillis = 15;
    public static boolean disableTelem = false;

    public static double intakeCurrentDraw = 6;
    public static double TICKS_PER_INCH = 1725;//1892.36864358;
    public static double X_ROTATE=6;
    public static double Y_ROTATE=-7;
    public static boolean reverseY = true;
    public static boolean reverseX = false;
    public static double headingOffset = 0;

    public static double bucketInitPos = 1;
    public static double dronePos = 0;

    public static double singleDropMillis = 150;

    public static boolean enableTelemetry = true;

    public static double reverseIntakePower = 0.4;
    int c = 0;
    public static int relatchthresh = 0;

    public static double heading = 0;

    public enum states {
        INTAKE,
        OUTTAKE,
        UNDERPASS,
        INIT,
        EXTENDO,
        FOLDEDFAR,
        DROP,
        NONE,
        TRYING_TO_FIX_EXTENDO,
        HANG
    }

    public states transferStates = states.NONE;

    public enum intakePos {
        EXTENDED,
        BUCKET_FOLDED,
        TILTING,
        SLIDES_FOLDED,

        RETRACTED,
        HANG
    }
    public intakePos intakeProgress = intakePos.RETRACTED;
    public enum outtakePos {
        RETRACTED,
        TILTED,
        SLIDES_EXTENDED,
        EXTENDED,
        DROP1,
        DROP2,
        NONE

    }

    public enum extendoPos {
        RETRACTED,
        TILTED,
        SLIDES_EXTENDED,
        EXTENDED,
        DROP1,
        DROP2,
        NONE

    }
    public enum tiltPos {
        MOVEBUCKET,
        WORKING,
        DONE
    }
    public tiltPos tiltProgess = tiltPos.MOVEBUCKET;

    public outtakePos outtakeProgress = outtakePos.RETRACTED;
    public extendoPos extendoProgress = extendoPos.RETRACTED;
    public outtakePos dropProgress = outtakePos.NONE;
    public intakePos hangProgress = intakePos.RETRACTED;
    public double intakeStartTime = 0;
    public ElapsedTime intakeTimer = new ElapsedTime();
    public ElapsedTime stateTimer = new ElapsedTime();

    public ElapsedTime outtakeTimer = new ElapsedTime();
    public ElapsedTime underpassTimer = new ElapsedTime();
    public Telemetry importantTelemetry;
    public TelemetryPacket telemetryPacket=new TelemetryPacket();
    public Odometry odometry;
    public ToggleTelemetry unimportantTelemetry;


    public int pixels = 0;
    public ElapsedTime time = new ElapsedTime();
    public static boolean headingLockIsFlipped = false;

    boolean intakeFull;

    public boolean done;
    public boolean lastIntake = false;
    public boolean lockForever = false;

    // small value controller
    public static double kPosp = 0.06;
    public static double kPosi = 0;
    public static double kPosd = 0;
    public static double onePixelIntakePower = 0.7;
    // large value: something like (0.1, 0, 0.05)
    public static double movePower = 0.5;

    public static double moveMultiplier = 1;
    public PIDController poscontroller = new PIDController(kPosp,kPosi,kPosd);
    states previous = states.NONE;
    int pixelsToDrop = 0;

    public static double intakeFlipDown = 0.7;
    public static double intakeFlipUp = 0.4;
    public UltrasonicSensor backdist;
    public static int underpassTime = 200;
    public void initialize() {


        allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        }

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        if (disableTelem) {
            telemetry = new MultipleTelemetry();
        }
        unimportantTelemetry = new ToggleTelemetry(telemetry);
        unimportantTelemetry.setEnabled(enableTelemetry);
        imu = hardwareMap.get(IMU.class, "imu 1");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);
        swerve = new SwerveController(hardwareMap, unimportantTelemetry);
        swerve.setVoltageLimit(0);
        odometry = new Odometry(hardwareMap, unimportantTelemetry);
        bucket = new Bucket(hardwareMap);
        intake = new Intake(hardwareMap);
        slides = new Slides(hardwareMap);
        drone = hardwareMap.get(Servo.class,"drone");
        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        backdist = new UltrasonicSensor(hardwareMap,"backdist");
        swerve.setLimits(1, 1);
        intake.endBrake();



        swerve.setInit();
    }
    public void resetIMU() {
        imu.resetYaw();
    }
    public boolean smartIntake() {
        if (transferStates!=states.INTAKE||intakeProgress!=intakePos.RETRACTED) {
            return false;
        }
        intake.armDown();
        lockForever = false;
        if (!lastIntake) {
            lockForever = false;
        }
        if (lockForever) {
            intake.intake(-reverseIntakePower);
            bucket.latch();
            gamepad1.rumble(50);
            intakeFull = true;
            return true;
        }
        else {
            if (pixels>1) {
                if (intakeTimer.milliseconds()>pixelInMillis) {

                    bucket.latch();
                    gamepad1.rumble(50);
                    intakeFull = true;
                    lockForever = true;
                }
                if (intakeTimer.milliseconds()>pixelInMillis+100) {
                    intake.intake(-0.5);
                }
                else {
                    bucket.unLatch();
                    intake.intake(onePixelIntakePower);
                }
                return true;
            }
            else {
                bucket.unLatch();
                if (pixels == 1) {
                    intake.intake(onePixelIntakePower);
                } else {
                    intake.intake(1);
                }
                intakeTimer.reset();
                intakeFull = false;
                lockForever = false;

                return false;
            }
        }

    }


    public void update() {

        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }
        pixels = bucket.update();

        heading = (imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)+headingOffset)%(2*Math.PI);
        odometry.update(getHeading());
        if (heading < 0) {
            heading += 2*Math.PI;
        }
        done = false;
        switch (transferStates) {
            case INTAKE:


                switch (intakeProgress) {
                    case EXTENDED:

                        // start timer, close bucket, and transition
                        stateTimer.reset();
                        bucket.intake();
                        intakeProgress = intakePos.BUCKET_FOLDED;
                        break;
                    case BUCKET_FOLDED:
                        //slides.tilt(intakeTilt);
                        // transition when slide is at correct pos, start moving slides when bucket is folded (aka wait 100 ms)
                        if (stateTimer.milliseconds() > bucketIntakeWaitms) {
                            if (slides.pidWithTol(slideIntakeEncVal+100, slideIntakeEncTol)) {
                                stateTimer.reset();
                                intakeProgress = intakePos.TILTING;
                                break;
                            }
                        }
                        break;
                    case TILTING:
                        slides.tilt(1);
                        slides.pidSlides(slideIntakeEncVal);
                        if (Math.abs(slides.getPosition()-slideIntakeEncVal) <= slideIntakeEncTol) {

                            stateTimer.reset();
                            intakeProgress = intakePos.SLIDES_FOLDED;
                            break;
                        }
                    case SLIDES_FOLDED:
                        telemetry.addLine("slides folded");
                        // tilt slides and transition
                        slides.tilt(intakeTilt);

                        intakeProgress = intakePos.RETRACTED;
                        //swerve.headingLock(false, );
                        swerve.psuedoLock(true);
                        break;
                    case RETRACTED:
                        if (slides.pidWithTol(slideIntakeEncVal, slideIntakeEncTol, false)) {
                            done = true;
                        }
                        if (pixels == 2) {
                            bucket.latch();
                        }
                        break;
                }
                break;
            case OUTTAKE:
                switch (outtakeProgress) {
                    case RETRACTED:
                        bucket.latch();
                        // tilt slides and transition
                        slides.tilt(outtakeTilt);
                        outtakeProgress = outtakePos.TILTED;
                        break;
                    case TILTED:
                        // move slides to above certain position and transition
                        int slidepos = slides.getPosition();
                        if (slidepos > slideOuttakeEncPos) {
                            slides.moveSlides(0);
                            outtakeProgress = outtakePos.SLIDES_EXTENDED;
                            break;
                        }
                        slides.moveSlides(1);
                        break;
                    case SLIDES_EXTENDED:
                        // move bucket and transition
                        bucket.latch();
                        bucket.outtake();
                        outtakeProgress = outtakePos.EXTENDED;
                        done = true;
                        break;

                    case DROP1:
                        if (pixels>1) {
                            bucket.unLatch();
                            outtakeProgress = outtakePos.DROP2;
                        }
                        else {
                            bucket.unLatch();
                            outtakeProgress = outtakePos.EXTENDED;
                        }
                        break;
                    case DROP2:
                        if (!SwerveModule.compare(outtakeTimer.milliseconds(),0,100)) {
                            outtakeTimer.reset();
                        }
                        if (pixels<2||outtakeTimer.milliseconds()>singleDropMillis) {

                            bucket.latch();
                            outtakeProgress = outtakePos.EXTENDED;
                        }
                        else
                            bucket.unLatch();
                        break;



                }
                break;
            case UNDERPASS:
                switch(tiltProgess) {
                    case MOVEBUCKET:

                        bucket.setPosition(underpassBucketPos);
                        if (underpassTimer.milliseconds() > 100) {
                            tiltProgess = tiltPos.WORKING;
                        }
                        break;
                    case WORKING:
                        swerve.headingLock(true,Math.PI/2*(Robot.headingLockIsFlipped?-1:1));
                        bucket.latch();
                        slides.pidSlides(-underpassSlidePos);
                        slides.tilt(1);
                        tiltProgess = tiltPos.DONE;
                        break;
                    case DONE:
                        bucket.setPosition(underpassDoneBucketPos);
                        slides.pidSlides(-underpassSlidePos);
                        done = true;
                        break;

                }
                break;

            case INIT:
                bucket.latch();
                bucket.setPosition(bucketInitPos);
                slides.tilt(0);
                done = true;
                break;
            case FOLDEDFAR:
                bucket.latch();
                bucket.setPosition(bucketInitPos);
                slides.pidSlides(400);
                slides.tilt(0);
                done = true;
                break;
            case DROP:
                switch (dropProgress) {
                    case DROP1:
                        bucket.drop(pixelsToDrop);
                        slides.moveSlides(1);
                        dropProgress = outtakePos.DROP2;
                        outtakeTimer.reset();
                        break;
                    case DROP2:
                        if (outtakeTimer.seconds() > 0.1) {
                            slides.moveSlides(0);
                            transferStates = previous;
                        }
                        break;
                }
                break;
            case TRYING_TO_FIX_EXTENDO:
                int slidepos = slides.getPosition();
                if (slidepos < slideExtendoEncPos) {
                    slides.moveSlides(1);
                } else {
                    slides.moveSlides(0);
                    transferStates = states.EXTENDO;
                }
                break;
            case EXTENDO:
                switch (extendoProgress) {
                    case RETRACTED:
                        bucket.latch();
                        // tilt slides and transition
                        slides.tilt(1);
                        transferStates = states.TRYING_TO_FIX_EXTENDO;
                        extendoProgress = extendoPos.SLIDES_EXTENDED;
                        stateTimer.reset();
                        break;
                    case TILTED:
                        if (stateTimer.seconds() < 2) {
                            slides.moveSlides(1);
                        } else {
                            slides.moveSlides(0);
                            extendoProgress = extendoPos.SLIDES_EXTENDED;
                        }
                        break;
                    case SLIDES_EXTENDED:
                        // move bucket and transition
                        slides.moveSlides(0);
                        bucket.latch();
                        bucket.extendoOuttake();
                        extendoProgress = extendoPos.EXTENDED;
                        done = true;
                        break;
                    case DROP1:
                        if (pixels>1) {
                            bucket.unLatch();
                            extendoProgress = extendoPos.DROP2;
                        }
                        else {
                            bucket.unLatch();
                            extendoProgress = extendoPos.EXTENDED;
                        }
                        break;
                    case DROP2:
                        if (!SwerveModule.compare(outtakeTimer.milliseconds(),0,100)) {
                            outtakeTimer.reset();
                        }
                        if (pixels<2||outtakeTimer.milliseconds()>singleDropMillis) {

                            bucket.latch();
                            extendoProgress = extendoPos.EXTENDED;
                        }
                        else
                            bucket.unLatch();
                        break;


                }
                break;
            case HANG:


                switch (hangProgress) {
                    case RETRACTED:

                        // start timer, close bucket, and transition
                        stateTimer.reset();
                        bucket.intake();
                        hangProgress = intakePos.BUCKET_FOLDED;
                        break;
                    case BUCKET_FOLDED:
                        slides.tilt(intakeTilt);
                        // transition when slide is at correct pos, start moving slides when bucket is folded (aka wait 100 ms)
                        if (stateTimer.milliseconds() > bucketIntakeWaitms) {
                            if (slides.pidWithTol(slideHangRaiseEncPos, slideIntakeEncTol)) {
                                stateTimer.reset();
                                hangProgress = intakePos.SLIDES_FOLDED;
                                break;
                            }
                        }
                        break;
                    case SLIDES_FOLDED:
                        telemetry.addLine("slides folded");
                        // tilt slides and transition
                        slides.tilt(intakeTilt);

                        hangProgress = intakePos.EXTENDED;
                        swerve.psuedoLock(true);
                        break;
                    case EXTENDED:
                        //slides.pidSlides(slideHangRaiseEncPos);
                        bucket.setPosition(0);
                        done = true;
                        break;
                    case HANG:
                        slides.pidSlides(slideHangLowerEncPos);
                        bucket.setPosition(0);
                }
                break;



        }
    }
    public void drop() {
        pixelsToDrop = 1;
        previous = transferStates;
        transferStates = states.DROP;
        dropProgress = outtakePos.DROP1;
    }
    public void dropAll() {
        pixelsToDrop = 2;
        previous = transferStates;
        transferStates = states.DROP;
        dropProgress = outtakePos.DROP1;
    }
    public void outtake() {
        transferStates = states.OUTTAKE;
        outtakeProgress = outtakePos.RETRACTED;
    }
    public void extendo() {
        transferStates = states.EXTENDO;
        extendoProgress = extendoPos.RETRACTED;
        //transferStates = states.TRYING_TO_FIX_EXTENDO;
        //stateTimer.reset();
    }
    public void intake() {
        transferStates = states.INTAKE;
        intakeProgress = intakePos.EXTENDED;
    }
    public void underpass() {
        transferStates = states.UNDERPASS;
        tiltProgess = tiltPos.MOVEBUCKET;
        underpassTimer.reset();
    }

    public void hang() {
        transferStates = states.HANG;
        hangProgress = intakePos.RETRACTED;
    }

    public void hangPID() {
        if (transferStates == states.HANG && done) {
            hangProgress = intakePos.HANG;
        }
    }
    public static double getHeading() {
        return heading;
    }

    public double getLoopSeconds() {
        double s = time.seconds();
        time.reset();
        return s;
    }

    public boolean alliance;
    public boolean side;

    public void setAlliance(boolean isRed) {
        alliance = isRed;
        swerve.flipHeading(!isRed);
    }

    public void setSide(boolean isBackstage) {
        side = isBackstage;
    }

    public double tolerance;
    public double angletolerance;

    public boolean pidToPosition(double x, double y) {
        odometry.setTarget(x, y);
        Pose2d odopose = odometry.getPose();
        y *= (alliance?1:-1);
        double xErr = x-odopose.getX();
        double yErr = y-odopose.getY();
        double dist = Math.sqrt(xErr*xErr+yErr*yErr);
        double angle = Math.atan2(xErr,-yErr);
        double move = poscontroller.calculate(0,-dist) * moveMultiplier;
        move = Math.pow(Math.abs(move), movePower)*Math.signum(move);
        double xmove = Math.sin(angle) * move;
        double ymove = Math.cos(angle) * -move;
        swerve.driveFieldCentric(ymove,-xmove, 0);
        return dist<tolerance;
    }

    /**
     * @param heading In Radians.
     * @return Whether the robot position is within the tolerance or not.
     */
    /*public boolean pidToPosition(double x, double y, double heading) {
        swerve.headingLock(true, heading);
        double angleerror = Math.abs(getHeading()-heading);


        return pidToPosition(x,y)&&angleerror<angletolerance;
    }

    /**
     * The number of inches from the backdrop the robot should be.
     */
    /*public static double backdropAlignmentConstant = 2;
    public boolean gooberAlign(Gamepad gamepad) {
        Pose2d odopose = odometry.getPose();
        slides.pidSlides((int)((gamepad.touchpad_finger_1_y+1)*600 + 800));
        if (pidToPosition(odopose.getX()+backdist.getDistance()-backdropAlignmentConstant, -35.41-gamepad.touchpad_finger_1_x*8, Math.PI/2*(headingLockIsFlipped?-1:1))) {
            return true;
        }
        return false;
    }

    public Pose2d RCtoFC(Pose2d FCRobotPose, Pose2d RCcoords) {
        double y = RCcoords.getX();
        double x = -RCcoords.getY();
        double botheading = FCRobotPose.getRotation().getRadians();
        botheading = -botheading;
        double x2 = x*Math.cos(botheading)+y*Math.sin(botheading);
        double y2 = x*-Math.sin(botheading)+y*Math.cos(botheading);
        return new Pose2d(FCRobotPose.getX()+y2,FCRobotPose.getY()-x2,new Rotation2d(-botheading));
    }
}
*/
