package org.firstinspires.ftc.teamcode.OpModes.auton;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.Controllers.MichaelPID;
import org.firstinspires.ftc.teamcode.subsystems.GVF.GVFFollower;
import org.firstinspires.ftc.teamcode.subsystems.MathUtils.Pose2D;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.vision.TeamPropProcessor;
import org.firstinspires.ftc.teamcode.vision.UndistortProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.Arrays;

@Config
public abstract class AutonBase extends Robot {

    public int propPosition = 1;
    public TeamPropProcessor propProcessor;
    public AprilTagProcessor aprilTag;
    public UndistortProcessor undistort;
    public VisionPortal propPortal;
    public VisionPortal tagPortal;

    public Pose2d startPos = new Pose2d(0,0,new Rotation2d(0));
    RunPos runPosition;
    ScoreAmount scoringAmount;
    int state = 0;




    public GVFFollower pathfollower;

    public ElapsedTime timer = new ElapsedTime();
    enum RunPos {
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


    public void initAuton(RunPos runPos, ScoreAmount scoreAmount) {
        initialize();
        resetForStart();
        runPosition = runPos;
        scoringAmount = scoreAmount;
        switch (runPos) {
            case RED_BACKSTAGE: startPos = new Pose2d(12,-84,new Rotation2d(Math.PI/2)); break;
            case RED_AUDIENCE: startPos = new Pose2d(-12,-84,new Rotation2d(Math.PI/2)); break;
            case BLUE_BACKSTAGE: startPos = new Pose2d(12,84,new Rotation2d(-Math.PI/2)); break;
            case BLUE_AUDIENCE: startPos = new Pose2d(-12,-4,new Rotation2d(-Math.PI/2)); break;
        }
        propProcessor = new TeamPropProcessor(true);

        initVisionPortal(new ArrayList<>(Arrays.asList(propProcessor)));
        slides.reset();

        swerve.setAuton();

        transferStates = states.INIT;

        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
    }
    public void run() {
        while (!isStopRequested()) {
            //drive forward to spike
            if (state==0) {
                if (pidToPosition(12, -48)||timer.seconds()>1) {

                }
            }
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
        transferStates = states.FOLDEDFAR;
        ElapsedTime timer = new ElapsedTime();
    }

    /**
     * Will return (0, 0, 0) if no paths are active.
     * @return
     */

    public void initVisionPortal(ArrayList<VisionProcessor> processors) {
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        builder.setCamera(hardwareMap.get(WebcamName.class, "camera"));

        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(new Size(320, 240));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        builder.setAutoStopLiveView(false);

        for (VisionProcessor processor : processors) {
            builder.addProcessor(processor);
        }

        propPortal = builder.build();

    }
    public void initAprilTag() {

        // Create the AprilTag processor.

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
        builder.addProcessor(aprilTag);
        builder.addProcessor(undistort);

        // Build the Vision Portal, using the above settings.
        tagPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        tagPortal.setProcessorEnabled(undistort, true);
        tagPortal.setProcessorEnabled(aprilTag, true);

    }

}