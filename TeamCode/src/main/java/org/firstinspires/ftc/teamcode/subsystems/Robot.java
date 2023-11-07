package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drivers.AnalogEncoder;

import java.util.List;
@Config
public abstract class Robot extends LinearOpMode {
    //Drive
    public SwerveController swerve;

    //Subsystems
    public Bucket bucket;
    public Intake intake;
    public static IMU imu;
    public Slides slides;
    public List<LynxModule> allHubs;

    public Servo drone;

    public static double kHeadingP = 0.03;
    public static double kHeadingI = 0;
    public static double kHeadingD = 0;

    public static double kSlidesP = 0.0071;
    public static double kSlidesI = 0;
    public static double kSlidesD = 0.00015;

    public static double kSlidesStatic = 0.1;

    public static int slidesPos = 0;
    public static double intakeTilt = 0.35;
    public static double outtakeTilt = 0.7;

    public static int slideIntakeEncVal = 30;

    public static int slideIntakeEncTol = 40;
    public static int bucketIntakeWaitms = 30;
    public static int slideOuttakeEncPos = 1000;
    public static int slideExtendoEncPos = 2400;
    public static double pixelInMillis = 15;
    public static boolean disableTelem = false;

    public static double intakeCurrentDraw = 5;
    public static double TICKS_PER_INCH = 1892.36864358;
    public static double X_ROTATE=6;
    public static double Y_ROTATE=-7;
    public static boolean reverseY = true;
    public static boolean reverseX = false;
    public static double headingOffset = 0;

    public static double bucketInitPos = 1;
    public static double dronePos = 0;

    public enum states {
        INTAKE,
        OUTTAKE,
        UNDERPASS,
        INIT,
        EXTENDO,
        FOLDEDFAR,
        NONE
    }

    public states transferStates = states.NONE;

    public enum intakePos {
        EXTENDED,
        BUCKET_FOLDED,
        SLIDES_FOLDED,
        RETRACTED
    }
    public intakePos intakeProgress = intakePos.RETRACTED;
    public enum outtakePos {
        RETRACTED,
        TILTED,
        SLIDES_EXTENDED,
        EXTENDED,
        DROP1,
        DROP2

    }
    public enum tiltPos {
        WORKING,
        DONE
    }
    public tiltPos tiltProgess = tiltPos.WORKING;

    public outtakePos outtakeProgress = outtakePos.RETRACTED;
    public outtakePos extendoProgress = outtakePos.RETRACTED;
    public double intakeStartTime = 0;
    public ElapsedTime intakeTimer = new ElapsedTime();
    public ElapsedTime stateTimer = new ElapsedTime();
    public Telemetry importantTelemetry;
    public TelemetryPacket telemetryPacket=new TelemetryPacket();
    public Odometry odometry;

    public int pixels = 0;
    public ElapsedTime time = new ElapsedTime();
    public boolean headingLockIsFlipped = false;

    boolean intakeFull;
    public void initialize() {

         allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        if (disableTelem) {
            telemetry = new MultipleTelemetry();
        }
        imu = hardwareMap.get(IMU.class, "imu 1");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);
        swerve = new SwerveController(hardwareMap, telemetry);
        odometry = new Odometry(hardwareMap, telemetry);
        bucket = new Bucket(hardwareMap);
        intake = new Intake(hardwareMap);
        slides = new Slides(hardwareMap);
        drone = hardwareMap.get(Servo.class,"drone");


        gamepad1.setLedColor(255,255,0,10000);
        gamepad2.setLedColor(255,200,0,10000);
    }
    public void resetIMU() {
        imu.resetYaw();
    }
    public boolean smartIntake() {
        if (pixels>1) {
            if (intakeTimer.milliseconds()>pixelInMillis) {
                intake.intake(-0.5);
                bucket.latch();
                gamepad1.rumble(50);
                intakeFull = true;
            }
            else {
                bucket.unLatch();
                intake.intake(1);
            }
            return true;
        }
        else {
            bucket.unLatch();
            intake.intake(1);
            intakeTimer.reset();
            intakeFull = false;

            return false;
        }
    }


    public void update() {

        odometry.update(getHeading());

        switch (transferStates) {
            case INTAKE:


                switch (intakeProgress) {
                    case EXTENDED:

                        // start timer, close bucket, and transition
                        stateTimer.reset();
                        bucket.intake();
                        bucket.unLatch();
                        intakeProgress = intakePos.BUCKET_FOLDED;
                        break;
                    case BUCKET_FOLDED:
                        slides.tilt(intakeTilt);
                        // transition when slide is at correct pos, start moving slides when bucket is folded (aka wait 100 ms)
                        int slidepos = slides.getPosition();
                        if (slidepos < slideIntakeEncVal + slideIntakeEncTol && slidepos > slideIntakeEncVal - slideIntakeEncTol) {
                            stateTimer.reset();
                            intakeProgress = intakePos.SLIDES_FOLDED;
                            break;
                        }
                        if (stateTimer.milliseconds() > bucketIntakeWaitms) {
                            slides.pidSlides(slideIntakeEncVal);
                        }
                        break;
                    case SLIDES_FOLDED:
                        telemetry.addLine("slides folded");
                        // tilt slides and transition
                        slides.tilt(intakeTilt);
                        intakeProgress = intakePos.RETRACTED;
                        swerve.psuedoLock(true);
                        break;
                    case RETRACTED:
                        slides.pidSlides(slideIntakeEncVal);
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
                        if (pixels<2) {
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
                    case WORKING:
                        swerve.headingLock(true,Math.PI/2*(headingLockIsFlipped?-1:1));
                        bucket.intake();
                        bucket.latch();
                        slides.pidSlides(-50);
                        slides.tilt(1);
                        tiltProgess = tiltPos.DONE;
                        break;
                    case DONE:

                }
                break;
            case EXTENDO:
                switch (extendoProgress) {
                    case RETRACTED:
                        bucket.latch();
                        // tilt slides and transition
                        slides.tilt(1);
                        extendoProgress = outtakePos.TILTED;
                        break;
                    case TILTED:
                        // move slides to above certain position and transition
                        int slidepos = slides.getPosition();
                        if (slidepos > slideExtendoEncPos) {
                            slides.moveSlides(0);
                            extendoProgress = outtakePos.SLIDES_EXTENDED;
                            break;
                        }
                        slides.moveSlides(1);
                        break;
                    case SLIDES_EXTENDED:
                        // move bucket and transition
                        bucket.latch();
                        bucket.extendoOuttake();
                        extendoProgress = outtakePos.EXTENDED;
                        break;
                    case DROP1:
                        if (pixels>1) {
                            bucket.unLatch();
                            extendoProgress = outtakePos.DROP2;
                        }
                        else {
                            bucket.unLatch();
                            extendoProgress = outtakePos.EXTENDED;
                        }
                        break;
                    case DROP2:
                        if (pixels<2) {
                            bucket.latch();
                            extendoProgress = outtakePos.EXTENDED;
                        }
                        else
                            bucket.unLatch();
                        break;

                }
                break;
            case INIT:
                bucket.latch();
                bucket.setPosition(bucketInitPos);
                slides.pidSlides(100);
                slides.tilt(0);
                break;
            case FOLDEDFAR:
                bucket.latch();
                bucket.setPosition(bucketInitPos);
                slides.pidSlides(400);
                slides.tilt(0);
                break;

        }
    }
    public static double getHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)+headingOffset;
    }

    public double getLoopSeconds() {
        double s = time.seconds();
        time.reset();
        return s;
    }
}
