package org.firstinspires.ftc.teamcode.subsytems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drivers.ToggleTelemetry;

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
    public IMU imu;
    public List<LynxModule> allHubs;
    public VoltageSensor voltageSensor;

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

    /**
     * PID Config.
     */
    public static double kHeadingP = 0.05;
    public static double kHeadingI = 0;
    public static double kHeadingD = 0;

    /**
     * Telemetry.
     */
    ToggleTelemetry toggleableTelemetry;





    public void initialize() {
        allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
        toggleableTelemetry = new ToggleTelemetry(telemetry);
        arm = new Arm(hardwareMap);
        bucket = new Bucket(hardwareMap);
        swerve = new SwerveController(hardwareMap, toggleableTelemetry, this);

        imu = hardwareMap.get(IMU.class, "imu 1");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);

        voltageSensor = hardwareMap.voltageSensor.iterator().next();
    }

    public void setPoseEstimate(Pose2d pose) {
        // TODO: add odometry.reset here
        headingOffset = pose.getRotation().getRadians()-botHeading;
    }

    /**
     * USE THIS FUNCTION WHENEVER YOU WANT TO GET A BOT HEADING OR USE BOTHEADING. DO NOT USE THE IMU.
     */
    public double getHeading() {
        return botHeading = AngleUnit.normalizeRadians(imuReading + headingOffset);
    }

    public void update() {
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }
        imuReading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        botHeading = getHeading();

        long loopNanos = timer.nanoseconds() - lastLoopNanos;
        
        arm.update(loopNanos);
        bucket.update();

        lastLoopNanos = timer.nanoseconds();


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
