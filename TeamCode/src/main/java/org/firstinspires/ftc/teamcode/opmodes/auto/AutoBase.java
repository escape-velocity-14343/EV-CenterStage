package org.firstinspires.ftc.teamcode.opmodes.auto;

import android.util.Log;
import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.WhiteBalanceControl;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.teamcode.opmodes.testing.CycleTest;
import org.firstinspires.ftc.teamcode.pathutils.AutonomousWaypoint;
import org.firstinspires.ftc.teamcode.pathutils.Point;
import org.firstinspires.ftc.teamcode.subsystems.ArmIVK;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.vision.TeamPropProcessor;
import org.firstinspires.ftc.teamcode.vision.UndistortProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.Arrays;

@Config
public abstract class AutoBase extends Robot {

    /*
    Auto Config
     */
    public static double PURPLE_PIXEL_DROP_TIME = 0.5;
    public static double YELLOW_PIXEL_DROP_TIME = 1;
    public static double STACK_INTAKE_TIME = 0.5;
    public static double STACK_OUTTAKE_TIME = 0.5;

    public static int PURPLE_PIXEL_ARM_EXTENSION_VALUE = 200;
    public static int YELLOW_PIXEL_ARM_EXTENSION_VALUE = 1000;

    public static AutonomousWaypoint closeStack = new AutonomousWaypoint(-70.5, -36, 0);
    public static AutonomousWaypoint middleStack = new AutonomousWaypoint(-70.5, -24, 0);
    public static AutonomousWaypoint farStack = new AutonomousWaypoint(-70.5, -12, 0);

    public static AutonomousWaypoint farBackdropDrop = new AutonomousWaypoint(64.5, -32, 0)
            .setBlueOffset(0, -5, 0);
    public static AutonomousWaypoint middleBackdropDrop = new AutonomousWaypoint(64.5, -39, 0)
            .setBlueOffset(0, -5, 0);
    public static AutonomousWaypoint closeBackdropDrop = new AutonomousWaypoint(64.5, -43.5, 0)
            .setBlueOffset(0, -5, 0);

    public static AutonomousWaypoint audiencePurpleDrop = new AutonomousWaypoint(0.5, -33, 0)
            .setAudienceOffset(-48, 0, 0);

    public static AutonomousWaypoint centerPurpleDrop = new AutonomousWaypoint(7, -24.5, 0)
            .setAudienceOffset(-50, 0, 0);

    public static AutonomousWaypoint backstagePurpleDrop = new AutonomousWaypoint(23.5, -30, 0)
            .setAudienceOffset(-48, 0, 0);
    public static double yellowHeightOffset = -2;
    public static double yellowDistOffset = -3;
    public static double intakeDist = 40;
    public static double intakeHeight = 0;
    public static double intakeLifterHeight = 24;
    public static double firstStackPickupBackOffset = 17;
    public static double purpleBackstageStartAudienceCaseBackOffset = 21;
    public static double stackFirstHeight = 2.3   ;
    public static double STACK_PICKUP_ONE_PROX = 50;
    public static double perPixelStackHeightDecrement = 0.5;
    public static double perCycleDropHeightIncrement = 3;
    public static double cycleSlideOffset = -17;
    public static double stackExtendoOffsetDegrees = 6;
    public static boolean cycle = false;
    public static int cameraWBtemp = 3000;
    //public static propPositions propPos = propPositions.CENTER;


    private VisionPortal propPortal;
    private TeamPropProcessor propProcessor;

    public enum propPositions {
        BACKSTAGE,
        CENTER,
        AUDIENCE
    }
    // if detection fails, run backstage because most effective
    public propPositions propPosition = propPositions.BACKSTAGE;

    //public StackDetectionProcessor stackFinder;
    //public AprilTagProcessor aprilTag;
    //public UndistortProcessor undistort;

    public VisionPortal tagPortal;

    // used for reference
    private boolean isRed = false;
    private boolean isBackstage = false;

    private Pose2d lastPose = new Pose2d();

    private ElapsedTime timer = new ElapsedTime();
    private int numPixelsInFarStack = 5;
    //private int cycles = 0;

    // state indexing: [major state] [substate] [small adjustment]
    // auto diagram:
    // 0xx - initial states
        // 0 - initial splitter state
        // 10 - delay state
    // 1xx - purple
        // 11x - audience
        // 12x - center
        // 13x - backstage
    // 2xx - yellow
        // 21x - audience
        // 22x - center
        // 23x - backstage
        // 29x - atag reset state
    // 3xx - go to cycle pos
        // 30x - cross stage door
        // 31x - cross middle truss
        // 32x - cross wall truss
        // 35x - backdrop to cycle
        // 36x - close stack, stage door
        // 37x - far stack, stage door
        // 38x - extendo cycle, middle truss
    // 4xx - cycle: intake
        // 40x - close stack
        // 41x - middle stack
        // 42x - far stack
        // 43x - close & middle stack
        // 45x - far stack: stage door path
        // 46x - middle stack: stage door path
    // 5xx - cycle: outtake
        // 50x - left
        // 51x - middle
        // 52x - right
        // 53x - right short
        // 54x - left short
        // 55x - backstage far
        // 56x - backstage wall
    // 9xx - park
        // 90x - backstage far
        // 91x - backstage wall
        // 95x - extendo park from scoring pos
        // 99x - failsafe states

    // TODO: stack intake tilt is 8.5 degrees, just use the lifter for the rest of the adjustments
    public int state = 10;



    enum park {
        WALL,
        FAR
    }

    /**
     * This function encapsulates everything you need to run the opmode. No other functions are necessary.
     */
    public void run(boolean isRed, boolean isBackstage) {

        initialize();
        odometry.resetYaw();
        //imu.resetYaw();

        this.isRed = isRed;
        this.isBackstage = isBackstage;


        AutonomousWaypoint.configAuto(isRed, isBackstage);
        swerve.setAuton();
        setFSMtoAuto();
        arm.reset();


        propProcessor = new TeamPropProcessor((isRed?TeamPropProcessor.RED:TeamPropProcessor.BLUE));

        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(false)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)


                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                .setLensIntrinsics(356.182925829, 356.182925829, 319.833258237, 235.480453978)

                // ... these parameters are fx, fy, cx, cy.

                .build();

        undistort = new UndistortProcessor();

        initPropPortal(undistort, propProcessor, aprilTag);

        int cycles = 2;

        double delay = 0;
        Gamepad lastgamepad1c = new Gamepad();
        park parkarea = park.FAR;

        while (opModeInInit()) {
            arm.tiltArm(160);
            bucket.tilt(1, 1);
            arm.setLifterHeight(0);

            update();
            //bucket.intake();
            bucket.latch();
            updatePropDetection();
            //propPosition = propPos;
            telemetry.addData("prop position", (propPosition==propPositions.BACKSTAGE?"BACKSTAGE":propPosition==propPositions.CENTER?"CENTER":"AUDIENCE"));
            // TODO: retune starting position
            setPoseEstimate(new AutonomousWaypoint(12, -64.8, Math.PI/2)
                    .setAudienceOffset(-48, 0, 0)
                    .setBlueHeadingReversed()
                    .getPoint().toPose2d());
            //setPoseEstimate(new Pose2d(-18, -12, new Rotation2d(0)));
            swerve.setVoltageCorrection(voltageSensor.getVoltage());

            if (gamepad1c.dpad_up&&!lastgamepad1c.dpad_up) {
                delay += 0.5;
            } else if (gamepad1c.dpad_down&&!lastgamepad1c.dpad_down) {
                delay -= 0.5;
                delay = Math.max(delay, 0);
            }
            if (gamepad1c.dpad_left) {
                parkarea = park.FAR;
            } else if (gamepad1c.dpad_right) {
                parkarea = park.WALL;
            }
            telemetry.addData("Delay Seconds", delay);
            telemetry.addData("Park Position", (parkarea==park.FAR?"Far":"Wall"));
            lastgamepad1c.copy(gamepad1c);
        }

        timer.reset();

        int cycleSlideExtension = 1700;
        int cyclenum = 0;
        //state = 371;
        bucket.tilt(0);

        // send to 2+0 state
        while (opModeIsActive()) {
            update();
            telemetry.addData("state", state);

            if (state == 10) {
                if (timer.seconds() > delay) {
                    setState(0);
                }
            } else if (state == 0) {
                switch (propPosition) {
                    case AUDIENCE:
                        setState(210, 115);
                        break;
                    case CENTER:
                        setState(120, 120);
                        break;
                    case BACKSTAGE:
                        setState(230, 130);
                        break;
                }

                // audience section
                // yellow
            } else if (state == 210) {
                goToPoint(new AutonomousWaypoint(40, -29.41, farBackdropDrop)
                        .setTolerances(0.5, 0.1));
                if (isStoppedAtPoint()) {
                    outtake();
                    setOuttake(AutonomousWaypoint.distance(odometry.getPose(), farBackdropDrop)+yellowDistOffset, 4+yellowHeightOffset);
                    setState(211);
                }
            } else if (state == 211) {
                resetToAprilTags();

                if (arm.isDone(10) || timer.seconds() < 2) {
                    setState(212);
                }
                // pause
            } else if (state == 212) {
                resetToAprilTags();
                if (arm.getVelocity() < 0.001 && timer.seconds() > 0.8) {
                    if (!isBackstage) {
                        bucket.unlatch();
                    }
                    bucket.setLeftLatch(false);
                }
                if (timer.seconds() > YELLOW_PIXEL_DROP_TIME) {
                    if (!isBackstage) {
                        bucket.unlatch();
                    }
                    bucket.setLeftLatch(false);
                    setState(110, 350);
                }
                // go to purple
            } else if (state == 110) {
                setFSMtoAuto();
                arm.tiltArm(173);
                arm.extendInches(3);
                bucket.tilt(0, 1);
                // point towards purple drop
                goToPoint(new AutonomousWaypoint(14, -36, audiencePurpleDrop)
                        .setAudienceOffset(-72, 0, 0));
                //.setBlueAudienceOffset(-48, 0, 0)
                //.setRotationOffset(Math.PI));
                if (atPoint()) {
                    //intake();
                    setState(111);
                }
                // move, tilt arm & drop
            } else if (state == 111) {
                arm.tiltArm(173);
                arm.extendInches(AutonomousWaypoint.distance(odometry.getPose(), audiencePurpleDrop)-4);
                bucket.tilt(0, 1);
                goToPoint(new AutonomousWaypoint(14, -36, audiencePurpleDrop)
                        .setTolerances(0.7, -0.1)
                        .setAudienceOffset(-50, 0, 0));
                if (arm.isDone(15)||timer.seconds()>2) {
                    setState(112);
                }
                // pause and transition
            } else if (state == 112) {
                if (arm.getVelocity() < 0.001) {
                    bucket.setRightLatch(false);
                }
                if (timer.seconds() > PURPLE_PIXEL_DROP_TIME) {
                    bucket.setRightLatch(false);
                    setState(350);
                }
            }
            // audience start audience prop
            else if (state == 115) {
                arm.tiltArm(173);
                arm.extendInches(3);
                bucket.tilt(0, 1);
                goToPoint(new AutonomousWaypoint(14, -40, audiencePurpleDrop)
                        .setTolerances(0.7, 0.1)
                        .setAudienceOffset(-50, 0, 0));

                if (atPoint()) {
                    swerve.stop();
                    setState(116);
                }
            } else if (state == 116) {
                swerve.stop();
                arm.tiltArm(173);
                arm.extendInches(AutonomousWaypoint.distance(odometry.getPose(), audiencePurpleDrop)-5);
                bucket.tilt(0, 1);
                if (arm.isDone(15)||timer.seconds()>2) {
                    setState(117);
                }
            } else if (state == 117) {
                if (arm.getVelocity() < 0.001) {
                    bucket.setRightLatch(false);
                }
                if (timer.seconds() > PURPLE_PIXEL_DROP_TIME) {
                    bucket.setRightLatch(false);
                    setState(370);
                }
            }

            // center
            // audience case
            else if (state == 125) {
                goToPoint(new AutonomousWaypoint(-38, -36, centerPurpleDrop));
                arm.extendInches(AutonomousWaypoint.distance(odometry.getPose(), centerPurpleDrop)-5);
                if (atPoint() && arm.isDone(10)) {
                    setState(126);
                }
            } else if (state == 126) {
                arm.tiltArm(173);
                bucket.tilt(0, 1);
                if (arm.isTilted(1)) {
                    bucket.setlatch(false, true);
                    setState(300);
                }
            } else if (state == 360) {
                setFSMtoAuto();
                //arm.tiltArm(10);
                goToPoint(new AutonomousWaypoint(-42,-36,closeStack)
                        .setRotationOffset(Math.PI)
                        .setHeadingTolerance(0.05)
                );
                if (isStoppedAtPoint()) {
                    setState(361);
                }
                if (timer.seconds() > 5) {
                    foldArm();
                    setState(365);
                }
            }
            else if (state == 361) {
                swerve.stop();

                ArmIVK.calcIVK(AutonomousWaypoint.distance(odometry.getPose(),closeStack)-firstStackPickupBackOffset, stackFirstHeight, Math.PI);
                goToArmIVK(20);
                arm.setLifterHeight(stackFirstHeight);
                bucket.smartLatch();
                if((arm.getVelocity()<0.011 || arm.isDone(10) || timer.seconds() > 2) && arm.isTilted(1)) {
                    setState(362);
                }
            }
            else if (state == 362) {
                bucket.smartLatch();
                bucket.tilt(ArmIVK.getBucketTilt(arm.getTilt(), Math.PI));
                if (timer.seconds()>3) {
                    arm.moveSlides(0);
                    setState(363);


                } else {
                    ArmIVK.calcIVK(ArmIVK.getLastDistance()+8*((double)(loopNanos)/1e9), ArmIVK.getLastHeight(), Math.PI);
                    forceGoToArmIVK();
                }
                if (bucket.getNumPixels()>1) {
                    arm.moveSlides(0);
                    setState(364);
                }
            }
            else if (state == 363) {
                setFSMtoAuto();
                arm.moveTilt(-0.2);
                bucket.tilt(ArmIVK.getBucketTilt(arm.getTilt(), Math.PI));
                bucket.smartLatch();
                if (timer.seconds()>2||bucket.getLeftDist() < STACK_PICKUP_ONE_PROX) {
                    setState(364);
                }
            } else if (state == 364) {
                bucket.smartLatch();

                if (timer.seconds() > 0.5) {
                    if (getState() != states.FOLDED) {
                        bucket.tilt(ArmIVK.getBucketTilt(arm.getTilt(), Math.PI));
                        arm.moveTilt(0);
                        foldArm();
                    }
                    if (arm.getTilt() > 162 && (arm.getPosition() < 50 || timer.seconds() > 2)) {
                        setFSMtoAuto();
                        arm.moveTilt(0);
                        // if we are already in the lane, skip the step
                        if (Math.abs(odometry.getPose().getY()) < 17) {
                            setState(366);
                        } else {
                            setState(365);
                        }
                    }
                }
            }
            else if (state == 365) {
                goToPoint(new AutonomousWaypoint(-60, -10,0).setTolerances(1,0.5));
                if (atPoint()) {
                    setState(366);
                }
            }
            else if (state == 366) {
                goToPoint(new AutonomousWaypoint(36, -10,0).setTolerances(1,0.5));
                if (atPoint()) {
                    setState(290);
                }
                if (timer.seconds() > 4) {
                    requestOpModeStop();
                }
            } else if (state == 370) {
                //setFSMtoAuto();
                //arm.tiltArm(10);
                goToPoint(new AutonomousWaypoint(-36,-12,farStack)
                        .setRotationOffset(Math.PI)
                        .setHeadingTolerance(0.05)
                );
                if (isStoppedAtPoint()) {
                    setState(371);
                }
                if (timer.seconds() > 5) {
                    foldArm();
                    setState(365);

                }
            }
            else if (state == 371) {
                bucket.unlatch();
                swerve.stop();

                ArmIVK.calcIVK(AutonomousWaypoint.distance(odometry.getPose(),farStack)-firstStackPickupBackOffset, stackFirstHeight, Math.PI);
                safeGoToArmIVK();
                arm.setLifterHeight(stackFirstHeight);
                //bucket.smartLatch();
                /*if (!isStoppedAtPoint()) {
                    foldArm();
                    setState(370);
                }*/
                if((arm.getVelocity()<0.011 || arm.isDone(10) || timer.seconds() > 2) && arm.isTilted(1)) {
                    setState(362);
                }
            } /*else if (state == 372) {
                goToPoint(new AutonomousWaypoint(-15, -12, farStack)
                        .setRotationOffset(Math.PI)
                        .setHeadingTolerance(0.05));
                if (isStoppedAtPoint()) {
                    setState(371);
                }
            }*/


            // move to purple and drop
            else if (state == 120) {
                arm.tiltArm(173);
                arm.extendInches(3);
                bucket.tilt(0, 1);
                goToPoint(new AutonomousWaypoint(14, -40, centerPurpleDrop)
                        .setTolerances(0.7, 0.1)
                        .setAudienceOffset(-50, 0, 0));

                if (atPoint()) {
                    swerve.stop();
                    setState(121);
                }
                // pause and transition
            } else if (state == 121) {
                swerve.stop();
                arm.tiltArm(173);
                arm.extendInches(AutonomousWaypoint.distance(odometry.getPose(), centerPurpleDrop)-5);
                bucket.tilt(0, 1);
                if (arm.isDone(15)||timer.seconds() > 2) {
                    setState(122);
                }
            }

            else if (state == 122) {
                swerve.stop();
                if (arm.getVelocity() < 0.001) {
                    bucket.setRightLatch(false);
                }
                if (timer.seconds() > PURPLE_PIXEL_DROP_TIME) {
                    bucket.setRightLatch(false);
                    setState(220, 360);
                }
                // score yellow
            } else if (state == 220) {
                arm.tiltArm(150);
                if (arm.getPosition() > 200 && !arm.isTilted(1)) {
                    arm.moveSlides(-0.7);
                } else {
                    //arm.moveTilt(0);
                    goToPoint(new AutonomousWaypoint(40, -39, middleBackdropDrop)
                            .setTolerances(0.5, 0.1));
                    if (isStoppedAtPoint()) {
                        swerve.stop();
                        outtake();
                        setOuttake(AutonomousWaypoint.distance(odometry.getPose(), middleBackdropDrop)+yellowDistOffset, yellowHeightOffset);
                        setState(221);
                    }
                }
            } else if (state == 221) {
                resetToAprilTags();
                swerve.stop();
                //arm.extend(YELLOW_PIXEL_ARM_EXTENSION_VALUE);
                Log.println(Log.INFO, "target extension", "target extension: " + ArmIVK.getSlideExtension());
                if (arm.isDone(10) || timer.seconds() > 2) {
                    setState(222);
                }
                // pause and transition
            } else if (state == 222) {
                resetToAprilTags();
                swerve.stop();
                if (arm.getVelocity() < 0.001 && timer.seconds() > 0.8) {
                    bucket.setRightLatch(false);
                    bucket.setLeftLatch(false);
                }
                if (timer.seconds() > YELLOW_PIXEL_DROP_TIME) {
                    bucket.setRightLatch(false);
                    bucket.setLeftLatch(false);
                    setState(350);
                }
            } // audience start yellow case
            else if (state == 225) {
                goToPoint(new AutonomousWaypoint(38, -12, 0, true));
                if (atPoint()) {
                    setState(226);
                }
            }
            else if (state == 226) {
                goToPoint(new AutonomousWaypoint(38, -35.41, middleBackdropDrop));
                if (atPoint()) {
                    outtake();
                    setOuttake(AutonomousWaypoint.distance(odometry.getPose(), middleBackdropDrop), 0);
                    // transition to wait and move to backdrop score state
                    setState(222);
                }
            }

            // backstage
            else if (state == 130) {
                setFSMtoAuto();
                //arm.moveSlides(0);
                arm.tiltArm(173);
                arm.extendInches(3);
                bucket.tilt(0, 1);
                goToPoint(new AutonomousWaypoint(8, -36, backstagePurpleDrop)
                        // account for truss positions
                        .setAudienceOffset(-46, 0, 0));
                if (atPoint() || timer.seconds() > 3) {
                    setState(131);
                }
            } else if (state == 131) {
                swerve.stop();

                arm.extendInches(AutonomousWaypoint.distance(odometry.getPose(), backstagePurpleDrop)-5);
                bucket.tilt(0, 1);
                if (arm.isDone(10) || (arm.getVelocity() < 0.001 && timer.seconds() > 0.5)) {
                    bucket.setRightLatch(false);
                    setState(132);
                }
            } else if (state == 132) {
                if (timer.seconds() > PURPLE_PIXEL_DROP_TIME) {
                    bucket.tilt(0);
                    setState(350, 360);
                }
            } else if (state == 230) {
                goToPoint(new AutonomousWaypoint(40, -42, closeBackdropDrop)
                        .setTolerances(0.5, 0.1));
                if (isStoppedAtPoint()) {
                    outtake();
                    setOuttake(AutonomousWaypoint.distance(odometry.getPose(), closeBackdropDrop)+yellowDistOffset, yellowHeightOffset);
                    setState(231);
                }
            } else if (state == 231) {
                resetToAprilTags();
                //arm.extend(YELLOW_PIXEL_ARM_EXTENSION_VALUE);
                if (arm.isDone(10) || timer.seconds() > 2) {
                    setState(232);
                }
            } else if (state == 232) {
                resetToAprilTags();
                if (arm.getVelocity() < 0.001 && timer.seconds() > 0.8) {
                    bucket.setLeftLatch(false);
                    if (!isBackstage) {
                        bucket.unlatch();
                    }
                }
                if (timer.seconds() > YELLOW_PIXEL_DROP_TIME) {
                    bucket.setLeftLatch(false);
                    if (!isBackstage) {
                        bucket.unlatch();
                    }
                    setState(130, 350);
                }
            }

            else if (state == 290) {
                goToPoint(new AutonomousWaypoint(40, -35.41, 0, true));
                if (isStoppedAtPoint()) {
                    setState(291);
                }
            } else if (state == 291) {
                resetToAprilTags();
                if (timer.seconds() > 1) {
                    if (propPosition==propPositions.CENTER) {
                        setState(220);
                    } else if (propPosition==propPositions.BACKSTAGE) {
                        setState(230);
                    } else {
                        setState(210);
                    }
                }
            }


            // stage door cross
            else if (state == 300) {
                goToPoint(new AutonomousWaypoint(-15, -12, 0, true));
                if (atPoint()) {
                    setState(301);
                }
            } else if (state == 301) {
                goToPoint(new AutonomousWaypoint(12+(propPosition==propPositions.AUDIENCE?3:0), -12, 0, true));
                if (atPoint()) {
                    switch (propPosition) {
                        case AUDIENCE:
                            setState(210);
                        case CENTER:
                            setState(225);
                        case BACKSTAGE:
                            setState(230);
                    }
                }
            }

            // go to normal cycle position
            else if (state == 350) {
                if (cycle) {
                    setState(380);
                    //setState(450);
                } else {
                    if (parkarea == park.FAR) {
                        setState(900);
                    } else {
                        setState(910);
                    }
                }
                /*if (arm.getPosition() > 100) {
                    arm.moveSlides(-1);
                } else {
                    arm.moveSlides(0);
                    arm.tiltArm(10);

                    // turn to stacks and go to scoring location
                    goToPoint(new AutonomousWaypoint(16, -36, closeStack)
                            .setTolerances(0.5, 0.1)
                            .setRotationOffset(Math.PI));
                    if (atPoint() && arm.isTilted(10)) {
                        if (cycle) {
                            setState(351);
                        } else {
                            setState(950);
                        }
                        //intake();
                        //setState(400);
                    }
                }*/
            }
            /*else if (state == 351) {
                goToPoint(new AutonomousWaypoint(-12,-36,closeStack)
                        .setHeadingTolerance(0.05)
                        .setRotationOffset(Math.PI));
                if (atPoint()) {
                    setFSMtoAuto();
                    ArmIVK.calcIVK(AutonomousWaypoint.distance(odometry.getPose(), closeStack) + intakeDist, intakeHeight, 0);

                    setState(400);
                }
            }*/

            else if (state == 380) {
                setFSMtoAuto();

                arm.extend(0);
                if (arm.isDone(10) || (arm.getVelocity() < 0.000001 && timer.seconds() > 0.5)) {
                    setState(381);
                }
            } else if (state == 381) {
                arm.tiltArm(10);
                if (arm.isTilted(3)) {
                    setState(382);
                }
            }
            else if (state == 382) {
                goToPoint(new AutonomousWaypoint(-12, -36, closeStack)
                        .setHeadingTolerance(Math.toRadians(1))
                        .setRotationOffset(Math.PI + Math.toRadians(stackExtendoOffsetDegrees))); // correct for slide angle
                if (isStoppedAtPoint()) {
                    setFSMtoAuto();
                    setState(400);
                }
            }

            // extendo cycle close stack
            else if (state == 400) {
                arm.tiltArm(CycleTest.tiltAngle);
                arm.setLifterHeight(0);
                if (arm.isTilted(CycleTest.tiltTolerance) && Math.abs(arm.getTiltVelocity()) < CycleTest.maxTiltVelo) {
                    setState(401);
                }
            } else if (state == 401) {
                arm.ultrasonicExtend(CycleTest.ultrasonicDistance+5);
                bucket.tilt(ArmIVK.getBucketTilt(arm.getTilt(), Math.PI));
                if (arm.getUltrasonicInches() < CycleTest.ultrasonicDistance+10) {
                    setState(402);
                }
            }else if (state == 402) {
                arm.setLifterHeight(CycleTest.lifterHeight);
                if (timer.seconds() > 0.5) {
                    setState(403);
                }
            } else if (state == 403) {
                arm.ultrasonicExtend(CycleTest.ultrasonicDistance);
                bucket.tilt(ArmIVK.getBucketTilt(arm.getTilt(), Math.PI));
                if (Math.abs(arm.getUltrasonicInches() - CycleTest.ultrasonicDistance) < CycleTest.ultrasonicTolerance && Math.abs(arm.getVelocity()) < CycleTest.maxArmVelo) {
                    setState(404);
                }
            } else if (state == 404) {
                arm.setLifterHeight(0);
                if (timer.seconds() > 1) {
                    setState(405);
                }
            } else if (state == 405) {
                bucket.latch();
                if (timer.seconds() > 0.5) {
                    setState(406);
                }
            } else if (state == 406) {
                bucket.tilt(0.5);
                arm.extend(0);
                if (timer.seconds() > 1 && (arm.isDone(20) || arm.getVelocity() < CycleTest.maxArmVelo)) {
                    setState(530);
                }
            }

            // cycle from stage door
            else if (state == 450) {
                foldArm();
                goToPoint(new AutonomousWaypoint(40, -12, farStack, true)
                        .setRotationOffset(Math.PI));
                if (atPoint()) {
                    setFSMtoAuto();
                    setState(451);
                }
            } else if (state == 451) {
                goToPoint(new AutonomousWaypoint(-36, -12, farStack)
                        .setRotationOffset(Math.PI)
                        .setTolerances(0.5, Math.toRadians(1.5)));
                if (isStoppedAtPoint()) {
                    setState(452);
                }
            } else if (state == 452) {
                swerve.stop();

                ArmIVK.calcIVK(AutonomousWaypoint.distance(odometry.getPose(),farStack)+cycleSlideOffset, stackFirstHeight-(5-numPixelsInFarStack)*perPixelStackHeightDecrement, Math.PI);
                Log.println(Log.INFO, "ARMIvk", "height: " + (stackFirstHeight-(5-numPixelsInFarStack)*perPixelStackHeightDecrement));
                safeGoToArmIVK();
                arm.setLifterHeight(stackFirstHeight-(5-numPixelsInFarStack)*perPixelStackHeightDecrement);
                bucket.smartLatch();
                if((arm.getVelocity()<0.011 || arm.isDone(10) || timer.seconds() > 2) && arm.isTilted(1)) {
                    setState(453);
                }
            }
            else if (state == 453) {
                bucket.smartLatch();
                bucket.tilt(ArmIVK.getBucketTilt(arm.getTilt(), Math.PI));
                if (timer.seconds()>3) {
                    arm.moveSlides(0);
                    setState(454);


                } else {
                    ArmIVK.calcIVK(ArmIVK.getLastDistance()+8*((double)(loopNanos)/1e9), ArmIVK.getLastHeight(), Math.PI);
                    forceGoToArmIVK();
                }
                if (bucket.getNumPixels()>0) {
                    arm.moveSlides(0);
                    setState(454);
                }
            }
            else if (state == 454) {
                arm.moveTilt(-0.2);
                bucket.tilt(ArmIVK.getBucketTilt(arm.getTilt(), Math.PI));
                bucket.smartLatch();
                if (timer.seconds()>2||bucket.getLeftDist() < STACK_PICKUP_ONE_PROX) {
                    setState(455);
                }
            } else if (state == 455) {
                bucket.smartLatch();

                if (timer.seconds() > 0.5) {
                    if (bucket.getNumPixels() > 0) {
                        cycles += 1;
                        numPixelsInFarStack -= 2;
                    }
                    bucket.tilt(0.5);
                    setState(540);
                    /*if (getState() != states.IFOLD) {
                        bucket.tilt(ArmIVK.getBucketTilt(arm.getTilt(), Math.PI));
                        arm.moveTilt(0);
                        foldArm();
                    }
                    if (arm.getTilt() > 162 && (arm.getPosition() < 50 || timer.seconds() > 2)) {
                        setFSMtoAuto();
                        arm.moveTilt(0);
                        setState(540);
                    }*/
                }
            }

            else if (state == 510) {
                outtake();
                setOuttake(AutonomousWaypoint.distance(odometry.getPose(), middleBackdropDrop), 2*cyclenum*1.44337567);
                setState(511);
            } else if (state == 511) {
               // if (arm.isStalled()) {
                    // this is dangerous and bad!!!
                 //   setState(512);
                //}
                if ((arm.isDone(10) || timer.seconds() > 2) && arm.isTilted(1)) {
                    setState(512);
                }
            } else if (state == 512) {
                // prevent further arm movement to ensure there is no pressure on the stack during outtake
                setFSMtoAuto();
                arm.holdPosition();
                bucket.dropFromStack();
                if (timer.seconds() > STACK_OUTTAKE_TIME) {
                    cyclenum++;
                    if (cyclenum >= cycles) {
                        setState(950);
                    } else {
                        setState(400);
                    }
                }

            }

            // right short
            else if (state == 530) {
                if (arm.getPosition() > 100) {
                    arm.moveSlides(-0.7);
                } else {
                    arm.moveSlides(0);
                    goToPoint(new AutonomousWaypoint(12, -36, getHeading()));
                    if (atPoint()) {
                        foldArm();
                        setState(531);
                    }
                }
            } else if (state == 531) {
                goToPoint(new AutonomousWaypoint(40, -40, 0));
                if (isStoppedAtPoint()) {
                    outtake();
                    setOuttake(AutonomousWaypoint.distance(odometry.getPose(), closeBackdropDrop)+yellowDistOffset+cycles*perCycleDropHeightIncrement/2, yellowHeightOffset+cycles*perCycleDropHeightIncrement);
                    setState(532);
                }
            } else if (state == 532) {
                resetToAprilTags();
                if (arm.isDone(10) || timer.seconds() > 2) {
                    setState(533);
                }
            } else if (state == 533) {
                resetToAprilTags();
                swerve.stop();
                if (arm.getVelocity() < 0.001 && timer.seconds() > 0.8) {
                    bucket.dropFromStack();
                }
                if (timer.seconds() > STACK_OUTTAKE_TIME) {
                    bucket.dropFromStack();
                    cycles++;
                    setState(400);
                }
            }

            else if (state == 540) {
                arm.tiltArm(10);
                goToPoint(new AutonomousWaypoint(40, -12, farStack, true)
                        .setRotationOffset(Math.PI));
                if (atPoint()) {
                    setState(541);
                }
            } else if (state == 541) {
                arm.tiltArm(10);
                goToPoint(new AutonomousWaypoint(40, -32, farBackdropDrop));
                if (isStoppedAtPoint()) {
                    setState(542);
                }
            } else if (state == 542) {
                resetToAprilTags();
                outtake();
                setOuttake(AutonomousWaypoint.distance(odometry.getPose(), farBackdropDrop)+yellowDistOffset+cycles*perCycleDropHeightIncrement/2, yellowHeightOffset+cycles*perCycleDropHeightIncrement);
                setState(543);
            } else if (state == 543) {
                resetToAprilTags();
                //arm.extend(YELLOW_PIXEL_ARM_EXTENSION_VALUE);
                if (arm.isDone(10) || timer.seconds() > 2) {
                    setState(544);
                }
            } else if (state == 544) {
                resetToAprilTags();
                if (arm.getVelocity() < 0.001 && timer.seconds() > 0.8) {
                    bucket.dropFromStack();
                }
                if (timer.seconds() > YELLOW_PIXEL_DROP_TIME) {
                    bucket.dropFromStack();
                    setState(450);
                }
            }

            else if (state == 900) {
                if (propPosition != propPositions.BACKSTAGE) {
                    setState(901);
                } else {
                    foldArm();
                    goToPoint(new AutonomousWaypoint(12, -12, 0));
                    if (atPoint()) {
                        setState(901);
                    }
                }
            } else if (state == 901) {
                foldArm();
                goToPoint(new AutonomousWaypoint(40, -12, 0));
                if (atPoint()) {
                    setState(902);
                }
            } else if (state == 902) {
                goToPoint(new AutonomousWaypoint(50, -12, 0));
                if (atPoint() || timer.seconds() > 5) {
                    requestOpModeStop();
                }
            }

            else if (state == 910) {
                if (propPosition != propPositions.BACKSTAGE) {
                    setState(911);
                } else {
                    foldArm();
                    goToPoint(new AutonomousWaypoint(12, -60, 0));
                    if (atPoint()) {
                        setState(911);
                    }
                }
            }
            else if (state == 911) {
                foldArm();
                goToPoint(new AutonomousWaypoint(40, -60, 0));
                if (atPoint()) {
                    setState(912);
                }
            } else if (state == 912) {
                goToPoint(new AutonomousWaypoint(50, -60, 0));
                if (atPoint() || timer.seconds() > 5) {
                    requestOpModeStop();
                }
            }

            // park state from backdrop scoring pos
            else if (state == 950) {
                goToPoint(new AutonomousWaypoint(16, -36, middleBackdropDrop));
                outtake();
                setOuttake(46, 3);
                setState(951);
            } else if (state == 951) {
                if (arm.isDone(10) && arm.isTilted(1)) {
                    requestOpModeStop();
                }
                if (arm.isStalled()) {
                    requestOpModeStop();
                }
            }

            // failsafe fold and stop state
            // use this state unless it is necessary to stop immediately as it folds the arm to reduce the risk of any robot crashing into it.
            // TODO: IMPLEMENT TILT STALL DETECTION SO WE DONT HAVE TO RELY ON TIMEOUTS PLEASE
            else if (state == 998) {
                swerve.stop();
                foldArm();
                if (armFSMIsDone() || timer.seconds() > 3) {
                    Log.println(Log.WARN, "AUTO", "Reached FOLDING STOP state. Stopping OpMode.");
                    if (timer.seconds() > 3) {
                        Log.println(Log.WARN, "AUTO", "Folding transition timed out. Potential contact during stoppage. Consider upping the time limit if no stall was observed.");
                    }
                    requestOpModeStop();
                }
            }

            // failsafe EMSTOP state
            else if (state == 999) {
                Log.println(Log.WARN, "AUTO", "Reached EMERGENCY STOP state. Stopping OpMode.");
                requestOpModeStop();
            }

            else {
                Log.println(Log.WARN, "AUTO", "Reached undefined state " + state + ". Stopping OpMode.");
                requestOpModeStop();
            }

            // if we are stalled emergency stop
            //if (swerveIsStalled()) {
            //    setState(998);
            //}

        }


    }

    public void setState(int state) {
        Log.println(Log.INFO, "auto state " + state, "heading: " + Math.toDegrees(getHeading()));
        this.state = state;
        swerve.stop();
        arm.moveSlides(0);
        arm.moveTilt(0);
        //setFSMtoAuto();
        lastPose = odometry.getPose();
        timer.reset();
    }

    public void setState(int backstageState, int audienceState) {
        setState(isBackstage?backstageState:audienceState);
    }

    /*
     * Vision
     */

    public void initPropPortal(VisionProcessor... processors) {
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        builder.setCamera(hardwareMap.get(WebcamName.class, "camera"));

        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);


        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        builder.setAutoStopLiveView(false);

        for (VisionProcessor processor : processors) {
            builder.addProcessor(processor);
        }

        propPortal = builder.build();

        for (VisionProcessor processor : processors) {
            propPortal.setProcessorEnabled(processor, true);
        }
        //FtcDashboard.getInstance().startCameraStream(propProcessor,0);

    }
    public void setWB(int temp) {
        propPortal.getCameraControl(WhiteBalanceControl.class).setWhiteBalanceTemperature(temp);
        propPortal.getCameraControl(WhiteBalanceControl.class).setMode(WhiteBalanceControl.Mode.MANUAL);
    }

    public void updatePropDetection() {
        int pos = propProcessor.getPosition();
        // orient blue detections correctly (left = backstage, center = center, right = audience)
        if (!isRed) {
            pos = 2 - pos;
        }
        propPosition = (pos==0?propPositions.AUDIENCE:pos==1?propPositions.CENTER:propPositions.BACKSTAGE);
        double[] sat = propProcessor.getVals();
        if (!isRed) {
            telemetry.addData("Backstage", sat[0]);
            telemetry.addData("Center",sat[1]);
            telemetry.addData("Audience",sat[2]);
        } else {
            telemetry.addData("Backstage", sat[2]);
            telemetry.addData("Center",sat[1]);
            telemetry.addData("Audience",sat[0]);
        }

    }

    AprilTagProcessor aprilTag;
    UndistortProcessor undistort;
    AprilTagLibrary tags = getCenterStageTagLibrary();
    public static double camxoffset = -3;
    public static double camyoffset = -3;

    public void resetToAprilTags() {
        // guard clause
        if (odometry.isMoving()) {
            return;
        }


        double x = 0;
        double y = 0;
        ArrayList<AprilTagDetection> detections = aprilTag.getFreshDetections();
        if (detections == null) {
            return;
        }
        for (AprilTagDetection detection : detections) {
            Pose2d pose = getFCPosition(detection, getHeading());
            x += pose.getX();
            y += pose.getY();

        }
        if (detections.size() > 0) {
            odometry.reset(x/detections.size(), y/detections.size());
        }

    }

    public Pose2d getFCPosition(AprilTagDetection detection, double botheading) {
        // TODO: change this back to detection.ftcPose.x once camera is swapped
        double x = detection.ftcPose.x-camxoffset;
        double y = detection.ftcPose.y-camyoffset;
        Log.println(Log.INFO, "ATAG", "Offset coords: (" + x + ", " + y + ")");
        botheading = -botheading;
        telemetry.addData("atag botheading", botheading);
        double x2 = x*Math.cos(botheading)+y*Math.sin(botheading);
        double y2 = x*-Math.sin(botheading)+y*Math.cos(botheading);
        Log.println(Log.INFO, "ATAG", "FC coordinates relative to the robot: (" + -y2 + ", " + x2 + ")");
        VectorF tagpose = tags.lookupTag(detection.id).fieldPosition;
        return new Pose2d(tagpose.get(0)-y2,tagpose.get(1)+x2,new Rotation2d(botheading));
    }

    public static AprilTagLibrary getCenterStageTagLibrary()
    {
        return new AprilTagLibrary.Builder()
                .addTag(1, "BlueAllianceLeft",
                        2, new VectorF(61.75f, 41.41f, 4f), DistanceUnit.INCH,
                        new Quaternion(0.3536f, -0.6124f, 0.6124f, -0.3536f, 0))
                .addTag(2, "BlueAllianceCenter",
                        2, new VectorF(61.75f, 35.41f, 4f), DistanceUnit.INCH,
                        new Quaternion(0.3536f, -0.6124f, 0.6124f, -0.3536f, 0))
                .addTag(3, "BlueAllianceRight",
                        2, new VectorF(61.75f, 29.41f, 4f), DistanceUnit.INCH,
                        new Quaternion(0.3536f, -0.6124f, 0.6124f, -0.3536f, 0))
                .addTag(4, "RedAllianceLeft",
                        2, new VectorF(61.75f, -29.41f, 4f), DistanceUnit.INCH,
                        new Quaternion(0.3536f, -0.6124f, 0.6124f, -0.3536f, 0))
                .addTag(5, "RedAllianceCenter",
                        2, new VectorF(61.75f, -35.41f, 4f), DistanceUnit.INCH,
                        new Quaternion(0.3536f, -0.6124f, 0.6124f, -0.3536f, 0))
                .addTag(6, "RedAllianceRight",
                        2, new VectorF(61.75f, -41.41f, 4f), DistanceUnit.INCH,
                        new Quaternion(0.3536f, -0.6124f, 0.6124f, -0.3536f, 0))
                .addTag(7, "RedAudienceWallLarge",
                        5, new VectorF(-70.25f, -40.625f, 5.5f), DistanceUnit.INCH,
                        new Quaternion(0.5f, -0.5f, -0.5f, 0.5f, 0))
                .addTag(8, "RedAudienceWallSmall",
                        2, new VectorF(-70.25f, -35.125f, 4f), DistanceUnit.INCH,
                        new Quaternion(0.5f, -0.5f, -0.5f, 0.5f, 0))
                .addTag(9, "BlueAudienceWallSmall",
                        2, new VectorF(-70.25f, 35.125f, 4f), DistanceUnit.INCH,
                        new Quaternion(0.5f, -0.5f, -0.5f, 0.5f, 0))
                .addTag(10, "BlueAudienceWallLarge",
                        5, new VectorF(-70.25f, 40.625f, 5.5f), DistanceUnit.INCH,
                        new Quaternion(0.5f, -0.5f, -0.5f, 0.5f, 0))
                .build();
    }

}

// old auton base
/*package org.firstinspires.ftc.teamcode.OpModes.auton;

import android.util.Log;
import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.MovingStatistics;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.GeneralMatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.teamcode.subsystems.Controllers.MichaelPID;
import org.firstinspires.ftc.teamcode.subsystems.GVF.GVFFollower;
import org.firstinspires.ftc.teamcode.subsystems.MathUtils.Pose2D;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.vision.StackDetectionProcessor;
import org.firstinspires.ftc.teamcode.vision.TeamPropProcessor;
import org.firstinspires.ftc.teamcode.vision.UndistortProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseRaw;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessorImpl;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.imgproc.Imgproc;
import org.openftc.apriltag.AprilTagDetectorJNI;
import org.openftc.apriltag.ApriltagDetectionJNI;

import java.util.ArrayList;
import java.util.Arrays;

@Config
public abstract class AutonBase extends Robot {

    public int propPosition = 1;
    public TeamPropProcessor propProcessor;
    public StackDetectionProcessor stackFinder;
    public AprilTagProcessor aprilTag;
    public UndistortProcessor undistort;
    public VisionPortal propPortal;
    public VisionPortal tagPortal;

    public Pose2d startPos = new Pose2d(0,0,new Rotation2d(0));
    RunPos runPosition;
    ScoreAmount scoringAmount;
    TravelPath travelPathway = TravelPath.WALL;
    Park parkingLot = Park.FAR;
    int state = 0;
    public static double intakeArmInitPos = 0.34;

    public static double intakeX = -61;

    Pose2d lastPose = new Pose2d();




    public GVFFollower pathfollower;

    public ElapsedTime timer = new ElapsedTime();
    public    enum RunPos {
        RED_AUDIENCE,
        BLUE_AUDIENCE,
        RED_BACKSTAGE,
        BLUE_BACKSTAGE
    }
    enum ScoreAmount {
        PURPLE,
        PRELOADS,
        PLUS_TWO,
        PLUS_FOUR
    }
    enum TravelPath {
        WALL,
        STAGE_DOOR
    }
    enum Park {
        WALL,
        FAR
    }

    AprilTagLibrary tags = AutonBase.getCenterStageTagLibrary();
    double camxoffset = 0.5;
    double camyoffset = -5.3;

    double oscillateRot = 0;
    public static int slideOuttakeEncAdd = 500;
    public static double angletol = 0.05;
    int cycles = 0;
    int newPropPos = 0;


    public void initAuton(RunPos runPos, ScoreAmount scoreAmount) {
        initialize();
        intake.armMove(intakeArmInitPos);

        resetForStart();
        runPosition = runPos;
        scoringAmount = scoreAmount;
        switch (runPos) {
            case RED_BACKSTAGE: startPos = new Pose2d(12,-65.28,new Rotation2d(Math.PI/2)); break;
            case RED_AUDIENCE: startPos = new Pose2d(-36,-65.28,new Rotation2d(Math.PI/2)); break;
            case BLUE_BACKSTAGE: startPos = new Pose2d(12,65.28,new Rotation2d(-Math.PI/2)); break;
            case BLUE_AUDIENCE: startPos = new Pose2d(-36,65.28,new Rotation2d(-Math.PI/2)); break;
        }
        propProcessor = new TeamPropProcessor(true);
        stackFinder = new StackDetectionProcessor();
        stackFinder.setAngle(AngleUnit.RADIANS.fromDegrees(70.42));

        initAprilTag();

        initVisionPortal(new ArrayList<>(Arrays.asList(propProcessor)));

        slides.reset();

        swerve.setAuton();
        swerve.setIQID();
        swerve.setLimits(1, 1);

        transferStates = states.INIT;

        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
    }

    // 2+0
    public void run() {

        while (!isStarted()) {
            update();

            updatePropReading();
            resetForStart();
            headingOffset=startPos.getRotation().getRadians();
            odometry.reset(startPos.getX(),startPos.getY());
            telemetry.addLine("Done Initializing");
            telemetry.update();
        }
        state=propPosition;
        timer.reset();
        tolerance = 1;
        angletolerance = angletol;
        intake.brake();
        intake.armDown();
        while(timer.milliseconds()<500)
            intake.pidIntake(0);

        if (runPosition == RunPos.BLUE_AUDIENCE || runPosition == RunPos.BLUE_BACKSTAGE) {
            propPosition = 2-propPosition;
        }
        if (runPosition==RunPos.RED_AUDIENCE||runPosition==RunPos.BLUE_AUDIENCE) {
            //state = -50;
        }
        double ymultiplier = (alliance?1:-1);
        odometry.reset(startPos.getX(),startPos.getY());
        newPropPos = propPosition;
        while (!isStopRequested()) {
            update();
            Pose2d odopose = odometry.getPose();

            //drive forward
            if (state==0) {
                intake.pidIntake(0);
                if (pidToPosition(startPos.getX()+4*(side?1:-1), -32, Math.PI/2, 1.5)) {
                    setState(5);
                }
            }
            else if (state==1) {
                intake.pidIntake(0);
                if (pidToPosition(startPos.getX(), -37, Math.PI/2, 1.5)) {
                    setState(5);
                }
            }
            else if (state==2) {
                if (pidToPosition(startPos.getX()+4*(side?1:-1), -32, Math.PI/2, 1.5)) {
                    setState(5);
                }
                intake.pidIntake(0);
            }
            // turn to spike
            else if (state==5) {
                intake.pidIntake(0);
                if (pidToPosition(odopose.getX(), odopose.getY()*ymultiplier, Math.PI/2*(2-propPosition), 0.5)) {
                    setState(6+propPosition);
                    intake.pidIntake(0);
                }
            }
            // move to spike
            else if (state==6) {
                if (pidToPosition(startPos.getX()-2, -32, Math.PI/2*(2-propPosition), 0.5)) {
                    setState(15);
                    tolerance = 0.5;
                }
            }
            else if (state==7) {
                if (pidToPosition(startPos.getX(), -37, Math.PI/2*(2-propPosition), 0.5)) {
                    setState(15);
                    tolerance = 0.5;
                }
            }
            else if (state==8) {
                if (pidToPosition(startPos.getX()+2, -32, Math.PI/2*(2-propPosition), 0.5)) {
                    setState(15);
                    tolerance = 0.5;
                }
            }
            /*//* move back 1 inch
        else if (state==10) {
        if (pidToPosition(startPos.getX()-2, -32, Math.PI/2*(2-propPosition), 2)) {
        setState(15);
        }
        }
        else if (state==11) {
        if (pidToPosition(startPos.getX(), -34, Math.PI/2*(2-propPosition), 2)) {
        setState(15);
        }
        }
        else if (state==12) {
        if (pidToPosition(startPos.getX() + 2, -32, Math.PI / 2 * (2 - propPosition), 2)) {
        setState(15);
        }
        }*/
        // outtake purple pixel
/*
        else if (state==15) {
        swerve.stop();
        double prev = Robot.kIntakeI;
        double prev2 = Robot.kIntakeP;
        if (!odometry.isMoving()) {
        //Robot.kIntakeI = 0;
        //Robot.kIntakeP = 0.012;
        intake.armUp();
        }
        if (timer.seconds() > 1) {
        intake.stop();
        setState(20);
        //Robot.kIntakeI = prev;
        //Robot.kIntakeP = prev2;
        if (runPosition==RunPos.RED_AUDIENCE||runPosition==RunPos.BLUE_AUDIENCE) {
        if (travelPathway==TravelPath.STAGE_DOOR)
        setState(21);
        else if (travelPathway==TravelPath.WALL)
        setState(25);
        }
        }
        }
        //travel cross field through stage door
        else if (state==21) {

        }
        //travel cross field through wall truss
        else if (state==25) {
        if (pidToPosition(-38,-45,getHeading(),1.5)) {
        setState(26);

        }
        }
        else if (state==26) {
        if (pidToPosition(-45,-60,Math.PI)) {
        if (transferStates != states.UNDERPASS) {
        underpass();
        }
        if (tiltProgess==tiltPos.DONE) {
        setState(27);
        }
        }
        if (timer.seconds() > 4) {
        requestOpModeStop();
        }
        }
        else if (state==27) {
        if (pidToPosition(30,-60,Math.PI,3)) {
        intake();
        setState(30);
        }
        }


        // move back in order to avoid spike marks
        else if (state==20) {
        if (pidToPosition(startPos.getX(), (lastPose.getY())*ymultiplier-4, lastPose.getRotation().getRadians(), 1)) {
        setState(22);
        if (propPosition != 2) {
        setState(40);
        }
        }
        }
        else if (state==22) {
        if (pidToPosition(startPos.getX(), -50, lastPose.getRotation().getRadians(), 0.5)) {
        setState(30);
        if (runPosition==RunPos.RED_AUDIENCE||runPosition==RunPos.BLUE_AUDIENCE) {
        if (travelPathway==TravelPath.WALL) {
        setState(25);
        }
        }
        }
        }
        // move horizantally to backdrop
        else if (state==30) {
        if (pidToPosition(36, -50, Math.PI, 1)) {
        setState(40);
        }
        }
        // move vertically to backdrop
        else if (state==40) {
        if (pidToPosition(46, -35.41+(1-newPropPos)*7.5, Math.PI, 1)) {
        setState(45);
        }
        }
        // relocalize
        else if (state==45) {
        int c = 0;
        if (!odometry.isMoving()) {
        if (atagRelocalize(getHeading())) {
        c += 1;
        }
        }
        if (c > 5 || timer.seconds() > 0.4) {
        setState(46);
        }
        }
        // move in front of scoring pos
        else if (state==46) {
        atagRelocalize(getHeading());
        if (pidToPosition(48, -35.41+(1-newPropPos)*7.5, Math.PI, 1)) {
        if (!odometry.isMoving()) {
        atagRelocalize(getHeading());
        }
        setState(47);
        }
        }
        // move to score
        else if (state==47) {
        atagRelocalize(getHeading());
        if (pidToPosition(52.5, -35.41+(1-newPropPos)*7.5, Math.PI, 1)) {

        setState(50);
        }
        }
        // extend slides
        else if (state==50) {
        atagRelocalize(getHeading());
        outtake();
        setState(60);
        }
        else if (state==60) {
        atagRelocalize(getHeading());
        if (pidSlidesWithTolerance(1100+cycles*250, 40, 1)) {
        setState(70);
        slides.moveSlides(0);
        }
        }
        // drop pixels
        else if (state==70) {
        atagRelocalize(getHeading());
        if (timer.seconds() > 0.5) {
        bucket.unLatch();
        }
        if (timer.seconds() > 0.8) {
        if (pidSlidesWithTolerance(1800+cycles*250, 40, 1)) {
        slides.moveSlides(0);
        setState(150);
        //outtake();
        if (cycles == 2) {
        setState(150);
        }
        }
        }
        }
        // cross stage door
        else if (state==75) {
        atagRelocalize(getHeading());
        if (timer.seconds() > 0.5) {
        underpass();
        }
        if (pidToPosition(24,-11, Math.PI,2)) {
        //underpass();
        setState(85);
        }
        }
        else if (state==80) {

        if (pidToPosition(-36,-11, Math.PI,2)) {
        intake();
        setState(85);
        }
        }
        // move to stack
        else if (state==85) {
        if (odopose.getX()<-34&&transferStates!=states.INTAKE) {
        intake();
        }
        if (pidToPosition(intakeX,-11, Math.PI,2)) {
        if (transferStates!=states.INTAKE) {
        intake();
        }
        setState(89);
        }
        }
        if (state==89) {
        if (done) {
        state=90;
        }
        }
        else if (state==90) {
        pidToPosition(intakeX-4+Math.sin(timer.seconds())*4,-11+Math.sin(timer.seconds()*2)*4,Math.PI + Math.sin(timer.seconds()*3)/2);
        if (timer.seconds() > 0.5) {
        if (smartIntake() || timer.seconds() > 6) {
        setState(95);
        underpass();
        intake.armUp();

        }
        }
        }
        // cross stage door
        else if (state==95) {
        intake.intake(-1);
        if (pidToPosition(-36,-11, Math.PI)||done) {
        intake.stop();
        setState(100);
        }
        }
        else if (state==100) {
        if (pidToPosition(24,-11, Math.PI,2)) {
        outtake();
        setState(46);
        cycles++;
        if (propPosition==0||propPosition==1) {
        newPropPos = 2;
        }
        if (propPosition==2) {
        newPropPos = 0;
        }
        }
        }
        // park & finish
        else if (state==150) {
        if (pidToPosition(44, -11, getHeading(), 5)) {
        swerve.stop();
        slides.moveSlides(0);
        intake.stop();
        requestOpModeStop();
        setState(1000);
        }
        }
        telemetry.update();

        }
        }

public void setState(int statenum) {
        state = statenum;
        timer.reset();
        swerve.stop();
        slides.moveSlides(0);
        lastPose = odometry.getPose();
        }

/**
 * @param heading In Radians.
 * @param timeout In Seconds.
 * @return Whether the timeout has ended or the robot position is within tolerance.
 */
    /*
public boolean pidToPosition(double x, double y, double heading, double timeout) {
        if (timer.seconds() > timeout) {
        return true;
        } else {
        return pidToPosition(x, y, heading);
        }
        }


public boolean pidSlidesWithTolerance(int target, int tolerance, double timeout) {
        if (timer.seconds() > timeout) {
        return true;
        } else {
        return slides.pidWithTol(target, tolerance);
        }
        }

public void updatePropReading() {
        try {
        propPosition = propProcessor.getPosition();
        double[] avgs = propProcessor.getVals();
        switch(propPosition) {
        case 0:
        telemetry.addData("Prop Pos", "left"); break;
        case 1:
        telemetry.addData("Prop Pos", "middle"); break;
        case 2:
        telemetry.addData("Prop Pos", "right"); break;
        }
        telemetry.addData("Left Avg:", avgs[0]);
        telemetry.addData("Middle Avg:", avgs[1]);
        telemetry.addData("Right Avg:", avgs[2]);

        } catch (Exception e) {}
        }

public void resetForStart() {
        odometry.reset();
        resetIMU();
        intake.reset();
        slides.reset();
        //transferStates = states.FOLDEDFAR;
        ElapsedTime timer = new ElapsedTime();
        }

/**
 * Will return (0, 0, 0) if no paths are active.
 * @return
 */
/*
public void initVisionPortal(ArrayList<VisionProcessor> processors) {
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        builder.setCamera(hardwareMap.get(WebcamName.class, "camera"));

        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(new Size(320, 240));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        builder.enableLiveView(false);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        builder.setAutoStopLiveView(false);

        for (VisionProcessor processor : processors) {
        builder.addProcessor(processor);
        }

        propPortal = builder.build();
        for (VisionProcessor processor : processors) {
        propPortal.setProcessorEnabled(processor, true);
        }

        }
public void initAprilTag() {

        // Create the AprilTag processor.

        /*aprilTag = new AprilTagProcessorImpl(356.182925829, 356.182925829, 319.833258237, 235.480453978, DistanceUnit.INCH, AngleUnit.DEGREES, AprilTagGameDatabase.getCenterStageTagLibrary(), false, true, true, true, AprilTagProcessor.TagFamily.TAG_36h11, 3) {
            @Override
            public Object processFrame(Mat input, long captureTimeNanos) {
                // Convert to greyscale
                Imgproc.cvtColor(input, this.grey, Imgproc.COLOR_RGBA2GRAY);

                synchronized (decimationSync)
                {
                    if(needToSetDecimation)
                    {
                        AprilTagDetectorJNI.setApriltagDetectorDecimation(nativeApriltagPtr, decimation);
                        needToSetDecimation = false;
                    }
                }

                // Run AprilTag
                detections = runAprilTagDetectorForMultipleTagSizes(captureTimeNanos);

                synchronized (detectionsUpdateSync)
                {
                    detectionsUpdate = detections;
                }

                // TODO do we need to deep copy this so the user can't mess with it before use in onDrawFrame()?
                return detections;
            }

            private MovingStatistics solveTime = new MovingStatistics(50);

            // We cannot use runAprilTagDetectorSimple because we cannot assume tags are all the same size
            ArrayList<AprilTagDetection> runAprilTagDetectorForMultipleTagSizes(long captureTimeNanos)
            {
                long ptrDetectionArray = AprilTagDetectorJNI.runApriltagDetector(nativeApriltagPtr, grey.dataAddr(), grey.width(), grey.height());
                if (ptrDetectionArray != 0)
                {
                    long[] detectionPointers = ApriltagDetectionJNI.getDetectionPointers(ptrDetectionArray);
                    ArrayList<AprilTagDetection> detections = new ArrayList<>(detectionPointers.length);

                    for (long ptrDetection : detectionPointers)
                    {
                        AprilTagMetadata metadata = tagLibrary.lookupTag(ApriltagDetectionJNI.getId(ptrDetection));

                        double[][] corners = ApriltagDetectionJNI.getCorners(ptrDetection);

                        Point[] cornerPts = new Point[4];
                        for (int p = 0; p < 4; p++)
                        {
                            cornerPts[p] = new Point(corners[p][0], corners[p][1]);
                        }

                        AprilTagPoseRaw rawPose;
                        AprilTagPoseFtc ftcPose;

                        if (metadata != null)
                        {
                            PoseSolver solver = poseSolver; // snapshot, can change

                            long startSolveTime = System.currentTimeMillis();

                            if (solver == PoseSolver.APRILTAG_BUILTIN)
                            {
                                double[] pose = ApriltagDetectionJNI.getPoseEstimate(
                                        ptrDetection,
                                        outputUnitsLength.fromUnit(metadata.distanceUnit, metadata.tagsize),
                                        fx, fy, cx, cy);

                                // Build rotation matrix
                                float[] rotMtxVals = new float[3 * 3];
                                for (int i = 0; i < 9; i++)
                                {
                                    rotMtxVals[i] = (float) pose[3 + i];
                                }

                                rawPose = new AprilTagPoseRaw(
                                        pose[0], pose[1], pose[2], // x y z
                                        new GeneralMatrixF(3, 3, rotMtxVals)); // R
                            }
                            else
                            {
                                Pose opencvPose = poseFromTrapezoid(
                                        cornerPts,
                                        cameraMatrix,
                                        outputUnitsLength.fromUnit(metadata.distanceUnit, metadata.tagsize),
                                        solver.code);

                                // Build rotation matrix
                                Mat R = new Mat(3, 3, CvType.CV_32F);
                                Calib3d.Rodrigues(opencvPose.rvec, R);
                                float[] tmp2 = new float[9];
                                R.get(0,0, tmp2);

                                rawPose = new AprilTagPoseRaw(
                                        opencvPose.tvec.get(0,0)[0], // x
                                        opencvPose.tvec.get(1,0)[0], // y
                                        opencvPose.tvec.get(2,0)[0], // z
                                        new GeneralMatrixF(3,3, tmp2)); // R
                            }

                            long endSolveTime = System.currentTimeMillis();
                            solveTime.add(endSolveTime-startSolveTime);
                        }
                        else
                        {
                            // We don't know anything about the tag size so we can't solve the pose
                            rawPose = null;
                        }

                        if (rawPose != null)
                        {
                            Orientation rot = Orientation.getOrientation(rawPose.R, AxesReference.INTRINSIC, AxesOrder.YXZ, outputUnitsAngle);

                            ftcPose = new AprilTagPoseFtc(
                                    rawPose.x,  // x   NB: These are *intentionally* not matched directly;
                                    rawPose.z,  // y       this is the mapping between the AprilTag coordinate
                                    -rawPose.y, // z       system and the FTC coordinate system
                                    -rot.firstAngle, // yaw
                                    rot.secondAngle, // pitch
                                    rot.thirdAngle,  // roll
                                    Math.hypot(rawPose.x, rawPose.z), // range
                                    outputUnitsAngle.fromUnit(AngleUnit.RADIANS, Math.atan2(-rawPose.x, rawPose.z)), // bearing
                                    outputUnitsAngle.fromUnit(AngleUnit.RADIANS, Math.atan2(-rawPose.y, rawPose.z))); // elevation
                        }
                        else
                        {
                            ftcPose = null;
                        }

                        double[] center = ApriltagDetectionJNI.getCenterpoint(ptrDetection);

                        detections.add(new AprilTagDetection(
                                ApriltagDetectionJNI.getId(ptrDetection),
                                ApriltagDetectionJNI.getHamming(ptrDetection),
                                ApriltagDetectionJNI.getDecisionMargin(ptrDetection),
                                new Point(center[0], center[1]), cornerPts, metadata, ftcPose, rawPose, captureTimeNanos));
                    }

                    ApriltagDetectionJNI.freeDetectionList(ptrDetectionArray);
                    return detections;
                }

                return new ArrayList<>();
            }

        };*/
        /*aprilTag = new AprilTagProcessor.Builder()
        .setDrawAxes(false)
        .setDrawCubeProjection(true)
        .setDrawTagOutline(true)
        .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
        .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
        .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)


        // == CAMERA CALIBRATION ==
        // If you do not manually specify calibration parameters, the SDK will attempt
        // to load a predefined calibration for your camera.
        .setLensIntrinsics(356.182925829, 356.182925829, 319.833258237, 235.480453978)

        // ... these parameters are fx, fy, cx, cy.

        .build();
        undistort = new UndistortProcessor();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        builder.setCamera(hardwareMap.get(WebcamName.class, "backcam"));

        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        builder.enableLiveView(false);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(undistort);
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        tagPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        tagPortal.setProcessorEnabled(undistort, true);
        tagPortal.setProcessorEnabled(aprilTag, true);

        }

/**
 * @param botheading In Radians.
 * @return FC Pose of bot.
 */
/*public Pose2d getFCPosition(AprilTagDetection detection, double botheading) {
        double x = detection.ftcPose.x-camxoffset;
        double y = detection.ftcPose.y-camyoffset;
        Log.println(Log.INFO, "ATAG", "Offset coords: (" + x + ", " + y + ")");
        botheading = -botheading;
        telemetry.addData("atag botheading", botheading);
        double x2 = x*Math.cos(botheading)+y*Math.sin(botheading);
        double y2 = x*-Math.sin(botheading)+y*Math.cos(botheading);
        Log.println(Log.INFO, "ATAG", "FC coordinates relative to the robot: (" + -y2 + ", " + x2 + ")");
        VectorF tagpose = tags.lookupTag(detection.id).fieldPosition;
        return new Pose2d(tagpose.get(0)+y2,tagpose.get(1)-x2,new Rotation2d(-botheading));
        }

/**
 * Relocalize using AprilTags.
 * @return True if AprilTags were found, otherwise false.
 */
/*public boolean atagRelocalize(double botHeading) {
        if (odometry.isMoving()) {
        return false;
        }
        ArrayList<AprilTagDetection> detections = aprilTag.getDetections();
        double totalX = 0, totalY = 0;
        int i = 0;
        for (AprilTagDetection detection : detections) {
        Pose2d FCPose = getFCPosition(detection, botHeading);
        totalX += FCPose.getX();
        totalY += FCPose.getY();
        i++;
        }
        if (i > 0) {
        totalX /= i;
        totalY /= i;

        Log.println(Log.INFO, "ATAG", "total x " + totalX);
        Log.println(Log.INFO, "ATAG", "total y " + totalY);
        Log.println(Log.INFO, "ATAG", "Old Coordinates: (" + odometry.getPose().getX() + ", " + odometry.getPose().getY() + ")");
        Pose2d odopose = odometry.getPose();
        if (Math.sqrt(Math.pow(totalX-odopose.getX(), 2)+Math.pow(totalY-odopose.getY(), 2)) < 10) {
        Log.println(Log.INFO, "ATAG", "Relocalizing!");

        odometry.reset(totalX, totalY);
        }
        return true;
        }
        return false;
        }

/**
 * Get the {@link org.firstinspires.ftc.vision.apriltag.AprilTagLibrary} for the Center Stage FTC game
 * @return the {@link org.firstinspires.ftc.vision.apriltag.AprilTagLibrary} for the Center Stage FTC game
 */
/*public static AprilTagLibrary getCenterStageTagLibrary()
        {
        return new AprilTagLibrary.Builder()
        .addTag(1, "BlueAllianceLeft",
        2, new VectorF(61.5f, 41.41f, 4f), DistanceUnit.INCH,
        new Quaternion(0.3536f, -0.6124f, 0.6124f, -0.3536f, 0))
        .addTag(2, "BlueAllianceCenter",
        2, new VectorF(61.5f, 35.41f, 4f), DistanceUnit.INCH,
        new Quaternion(0.3536f, -0.6124f, 0.6124f, -0.3536f, 0))
        .addTag(3, "BlueAllianceRight",
        2, new VectorF(61.5f, 29.41f, 4f), DistanceUnit.INCH,
        new Quaternion(0.3536f, -0.6124f, 0.6124f, -0.3536f, 0))
        .addTag(4, "RedAllianceLeft",
        2, new VectorF(61.5f, -29.41f, 4f), DistanceUnit.INCH,
        new Quaternion(0.3536f, -0.6124f, 0.6124f, -0.3536f, 0))
        .addTag(5, "RedAllianceCenter",
        2, new VectorF(61.5f, -35.41f, 4f), DistanceUnit.INCH,
        new Quaternion(0.3536f, -0.6124f, 0.6124f, -0.3536f, 0))
        .addTag(6, "RedAllianceRight",
        2, new VectorF(61.5f, -41.41f, 4f), DistanceUnit.INCH,
        new Quaternion(0.3536f, -0.6124f, 0.6124f, -0.3536f, 0))
        .addTag(7, "RedAudienceWallLarge",
        5, new VectorF(-70.25f, -40.625f, 5.5f), DistanceUnit.INCH,
        new Quaternion(0.5f, -0.5f, -0.5f, 0.5f, 0))
        .addTag(8, "RedAudienceWallSmall",
        2, new VectorF(-70.25f, -35.125f, 4f), DistanceUnit.INCH,
        new Quaternion(0.5f, -0.5f, -0.5f, 0.5f, 0))
        .addTag(9, "BlueAudienceWallSmall",
        2, new VectorF(-70.25f, 35.125f, 4f), DistanceUnit.INCH,
        new Quaternion(0.5f, -0.5f, -0.5f, 0.5f, 0))
        .addTag(10, "BlueAudienceWallLarge",
        5, new VectorF(-70.25f, 40.625f, 5.5f), DistanceUnit.INCH,
        new Quaternion(0.5f, -0.5f, -0.5f, 0.5f, 0))
        .build();
        }

        }*/