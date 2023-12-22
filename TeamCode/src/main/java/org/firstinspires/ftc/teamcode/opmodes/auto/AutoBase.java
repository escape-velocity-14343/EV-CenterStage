package org.firstinspires.ftc.teamcode.opmodes.auto;

import android.util.Log;
import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.pathutils.AutonomousWaypoint;
import org.firstinspires.ftc.teamcode.pathutils.Point;
import org.firstinspires.ftc.teamcode.subsystems.ArmIVK;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.vision.TeamPropProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;

import java.util.ArrayList;
import java.util.Arrays;

@Config
public abstract class AutoBase extends Robot {

    /*
    Auto Config
     */
    public static double PURPLE_PIXEL_DROP_TIME = 0.5;
    public static double YELLOW_PIXEL_DROP_TIME = 0.5;
    public static double STACK_INTAKE_TIME = 0.5;
    public static double STACK_OUTTAKE_TIME = 0.5;

    public static int PURPLE_PIXEL_ARM_EXTENSION_VALUE = 200;
    public static int YELLOW_PIXEL_ARM_EXTENSION_VALUE = 1000;

    public static AutonomousWaypoint closeStack = new AutonomousWaypoint(-70.5, -36, 0);
    public static AutonomousWaypoint middleStack = new AutonomousWaypoint(-70.5, -48, 0);
    public static AutonomousWaypoint farStack = new AutonomousWaypoint(-70.5, -60, 0);

    public static AutonomousWaypoint farBackdropDrop = new AutonomousWaypoint(64.5, -29.41, 0);
    public static AutonomousWaypoint middleBackdropDrop = new AutonomousWaypoint(64.5, -35.41, 0);
    public static AutonomousWaypoint closeBackdropDrop = new AutonomousWaypoint(64.5, -41.51, 0);

    public static AutonomousWaypoint audiencePurpleDrop = new AutonomousWaypoint(0.5, -29.51, 0)
            .setAudienceOffset(-48, 0, 0);


    public static AutonomousWaypoint centerPurpleDrop = new AutonomousWaypoint(14, -24.5, 0)
            .setAudienceOffset(-52, 0, 0);

    public static AutonomousWaypoint backstagePurpleDrop = new AutonomousWaypoint(23.5, -24.5, 0)
            .setAudienceOffset(-48, -2, 0);


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

    // state indexing: [major state] [substate] [small adjustment]
    // auto diagram:
    // 1xx - purple
        // 11x - audience
        // 12x - center
        // 13x - backstage
    // 2xx - yellow
        // 21x - audience
        // 22x - center
        // 23x - backstage
    // 3xx - go to cycle pos
        // 30x - cross stage door
        // 31x - cross middle truss
        // 32x - cross wall truss
        // 35x - backdrop to cycle
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
        // 55x - backstage far
        // 56x - backstage wall
    // 9xx - park
        // 90x - backstage far
        // 91x - backstage wall
        // 95x - extendo park from scoring pos
        // 99x - failsafe states
    public int state = 0;

    /**
     * This function encapsulates everything you need to run the opmode. No other functions are necessary.
     */
    public void run(boolean isRed, boolean isBackstage) {

        this.isRed = isRed;
        this.isBackstage = isBackstage;

        initialize();
        AutonomousWaypoint.configAuto(isRed, isBackstage);
        swerve.setAuton();
        setFSMtoAuto();


        propProcessor = new TeamPropProcessor((isRed?TeamPropProcessor.RED:TeamPropProcessor.BLUE));

        initPropPortal(propProcessor);

        int cycles = 3;

        while (opModeInInit()) {
            update();
            bucket.intake();
            updatePropDetection();
            // TODO: retune starting position
            setPoseEstimate(new AutonomousWaypoint(12, -64.8, Math.PI/2)
                    .setAudienceOffset(-24, 0, 0)
                    .setBlueHeadingReversed()
                    .getPoint().toPose2d());
        }

        timer.reset();

        int cycleSlideExtension = 2500;
        int cyclenum = 0;

        // send to 2+0 state
        while (opModeIsActive()) {
            update();
            if (state == 0) {
                switch (propPosition) {
                    case AUDIENCE:
                        setState(210, 110);
                        break;
                    case CENTER:
                        setState(120, 125);
                        break;
                    case BACKSTAGE:
                        setState(230, 130);
                        break;
                }

                // audience section
                // yellow
            } else if (state == 210) {
                goToPoint(new AutonomousWaypoint(12, -29.41, farBackdropDrop));
                if (atPoint()) {
                    outtake();
                    setOuttake(AutonomousWaypoint.distance(odometry.getPose(), farBackdropDrop), 0);
                    setState(211);
                }
            } else if (state == 211) {
                // TODO: emperically tune the arm target
                // arm.extend(YELLOW_PIXEL_ARM_EXTENSION_VALUE);
                if (arm.isDone(10)) {
                    bucket.setlatch(false, true);
                    setState(212);
                }
                // pause
            } else if (state == 212) {
                if (timer.seconds() > YELLOW_PIXEL_DROP_TIME) {
                    setState(110);
                }
                // go to purple
            } else if (state == 110) {
                // point towards purple drop
                goToPoint(new AutonomousWaypoint(-12, -29.41, audiencePurpleDrop)
                        .setBlueAudienceOffset(-48, 0, 0)
                        .setRotationOffset(Math.PI));
                if (atPoint()) {
                    setState(111);
                }
                // move, tilt arm & drop
            } else if (state == 111) {
                arm.extendInches(AutonomousWaypoint.distance(odometry.getPose(), audiencePurpleDrop));
                arm.tiltArm(180);
                bucket.intake();
                if (arm.isTilted(2) && arm.isDone(10)) {
                    bucket.unlatch();
                    setState(112);
                }
                // pause and transition
            } else if (state == 112) {
                if (timer.seconds() > PURPLE_PIXEL_DROP_TIME) {
                    setState(350, 300);
                }
            }

            // center
            // audience case
            else if (state == 125) {
                goToPoint(new AutonomousWaypoint(-14, -14, centerPurpleDrop)
                        .setRotationOffset(Math.PI));
                arm.extendInches(AutonomousWaypoint.distance(odometry.getPose(), centerPurpleDrop));
                if (atPoint() && arm.isDone(10)) {
                    setState(126);
                }
            } else if (state == 126) {
                arm.tiltArm(180);
                bucket.intake();
                if (arm.isTilted(1)) {
                    bucket.setlatch(false, true);
                    setState(300);
                }
            }

            // move to purple and drop
            else if (state == 120) {
                goToPoint(new AutonomousWaypoint(14, -35.76, centerPurpleDrop));
                arm.extendInches(AutonomousWaypoint.distance(odometry.getPose(), centerPurpleDrop));
                if (atPoint() && arm.isDone(10)) {
                    bucket.setlatch(false, true);
                    setState(121);
                }
                // pause and transition
            } else if (state == 121) {
                if (timer.seconds() > PURPLE_PIXEL_DROP_TIME) {
                    setState(220, 300);
                }
                // score yellow
            } else if (state == 220) {
                goToPoint(new AutonomousWaypoint(12, -35.41, middleBackdropDrop));
                if (atPoint()) {
                    outtake();
                    setOuttake(AutonomousWaypoint.distance(odometry.getPose(), middleBackdropDrop), 0);
                    setState(221);
                }
            } else if (state == 221) {
                //arm.extend(YELLOW_PIXEL_ARM_EXTENSION_VALUE);
                if (arm.isDone(10)) {
                    bucket.unlatch();
                    setState(222);
                }
                // pause and transition
            } else if (state == 222) {
                if (timer.seconds() > YELLOW_PIXEL_DROP_TIME) {
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
                goToPoint(new AutonomousWaypoint(12, -24.5, backstagePurpleDrop)
                        // account for truss positions
                        .setAudienceOffset(-48, -2, 0));
                if (atPoint()) {
                    setState(131);
                }
            } else if (state == 131) {
                arm.extendInches(AutonomousWaypoint.distance(odometry.getPose(), backstagePurpleDrop));
                if (arm.isDone(10)) {
                    bucket.setlatch(false, true);
                    setState(132);
                }
            } else if (state == 132) {
                if (timer.seconds() > PURPLE_PIXEL_DROP_TIME) {
                    setState(230, 300);
                }
            } else if (state == 230) {
                goToPoint(new AutonomousWaypoint(12, -41.41, closeBackdropDrop));
                if (atPoint()) {
                    outtake();
                    setOuttake(AutonomousWaypoint.distance(odometry.getPose(), closeBackdropDrop), 0);
                    setState(231);
                }
            } else if (state == 231) {
                //arm.extend(YELLOW_PIXEL_ARM_EXTENSION_VALUE);
                if (arm.isDone(10)) {
                    bucket.unlatch();
                    setState(232);
                }
            } else if (state == 232) {
                if (timer.seconds() > YELLOW_PIXEL_DROP_TIME) {
                    setState(350);
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
                foldArm();
                // turn to stacks and go to scoring location
                goToPoint(new AutonomousWaypoint(8, -36, closeStack)
                        .setHeadingTolerance(0.05)
                        .setRotationOffset(Math.PI));
                if (atPoint()) {
                    intake();
                    setState(400);
                }
            }

            // cycle close stack
            else if (state == 400) {
                setIntake(175);

                arm.extend(cycleSlideExtension);
                if (arm.isStalled() && arm.getPosition() > 2300) {
                    cycleSlideExtension = arm.getPosition();
                }
                if (arm.isDone(10)) {
                    setIntake(178);
                    setState(401);
                }
            } else if (state == 401) {
                if (arm.isTilted(1)) {
                    if (bucket.getNumPixels() > 0) {
                        bucket.smartLatch();
                        setState(402);
                    } else {
                        setState(950);
                    }
                }
            } else if (state == 402) {
                if (timer.seconds() > STACK_INTAKE_TIME) {
                    setState(403);
                }
            } else if (state == 403) {
                setIntake(175);
                arm.extend(100);
                if (arm.isDone(10) && arm.isTilted(1)) {
                    setState(510);
                }
            }

            else if (state == 510) {
                outtake();
                setOuttake(AutonomousWaypoint.distance(odometry.getPose(), middleBackdropDrop), 2*cyclenum*1.44337567);
                setState(511);
            } else if (state == 511) {
                if (arm.isStalled()) {
                    // this is dangerous and bad!!!
                    ArmIVK.setSlideExtension(arm.getPosition());
                }
                if (arm.isDone(10) && arm.isTilted(1)) {
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

            // park state from backdrop scoring pos
            else if (state == 950) {
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
            if (swerveIsStalled()) {
                setState(998);
            }

        }


    }

    public void setState(int state) {
        this.state = state;
        swerve.stop();
        arm.moveSlides(0);
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

    public void updatePropDetection() {
        int pos = propProcessor.getPosition();
        // orient blue detections correctly (left = backstage, center = center, right = audience)
        if (!isRed) {
            pos = 2 - pos;
        }
        propPosition = (pos==0?propPositions.AUDIENCE:pos==1?propPositions.CENTER:propPositions.BACKSTAGE);
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