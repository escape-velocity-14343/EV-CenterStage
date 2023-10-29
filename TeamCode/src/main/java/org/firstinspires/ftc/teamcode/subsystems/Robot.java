package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drivers.AnalogEncoder;

import java.util.List;
@Config
public abstract class Robot extends LinearOpMode {
    //Drive
    public SwerveController swerve;

    //Subsystems
    public Bucket bucket;
    public Intake intake;
    public IMU imu;
    public Slides slides;
    public List<LynxModule> allHubs;

    public static double kHeadingP = 0.02;
    public static double kHeadingI = 0;
    public static double kHeadingD = 0;

    public static double kSlidesP = 0.011;
    public static double kSlidesI = 0;
    public static double kSlidesD = 0.00015;

    public static double kSlidesStatic = 0.15;

    public static int slidesPos = 0;
    public static double intakeTilt = 0.35;
    public static double outtakeTilt = 0.7;

    public static int slideIntakeEncVal = 40;

    public static int slideIntakeEncTol = 10;
    public static int bucketIntakeWaitms = 30;
    public static int slideOuttakeEncPos = 700;

    public enum states {
        INTAKE,
        OUTTAKE,
        UNDERPASS,
        INIT
    }

    public states transferStates = states.INIT;

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
        EXTENDED

    }
    public outtakePos outtakeProgress = outtakePos.RETRACTED;
    public double intakeStartTime = 0;
    public ElapsedTime timer = new ElapsedTime();
    public ElapsedTime stateTimer = new ElapsedTime();
    public void initialize() {

         allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        imu = hardwareMap.get(IMU.class, "imu 1");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);
        swerve = new SwerveController(hardwareMap, telemetry);
        bucket = new Bucket(hardwareMap);
        intake = new Intake(hardwareMap);
        slides = new Slides(hardwareMap);

    }
    public void resetIMU() {
        imu.resetYaw();
    }
    public boolean smartIntake(int pixels) {
        if (pixels>1) {
            intake.intake(-0.5);
            bucket.latch();
            gamepad1.rumble(20);
            return true;
        }
        else {
            bucket.unLatch();
            intake.intake(1);

            return false;
        }
    }

    public void update() {
        switch (transferStates) {
            case INTAKE:
                switch(intakeProgress) {
                    case EXTENDED:
                        // start timer, close bucket, and transition
                        stateTimer.reset();
                        bucket.intake();
                        bucket.unLatch();
                        intakeProgress = intakePos.BUCKET_FOLDED;
                        break;
                    case BUCKET_FOLDED:
                        // transition when slide is at correct pos, start moving slides when bucket is folded (aka wait 100 ms)
                        int slidepos = slides.getPosition();
                        if (slidepos < slideIntakeEncVal+slideIntakeEncTol && slidepos > slideIntakeEncVal-slideIntakeEncTol) {
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
                        break;
                }
                break;
            case OUTTAKE:
                switch (outtakeProgress) {
                    case RETRACTED:
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

                }
                break;
            case UNDERPASS:
                bucket.intake();
                slides.pidSlides(0);
                slides.tilt(1);
                break;

        }
    }

}
