package org.firstinspires.ftc.teamcode.OpModes.auton;

import android.util.Log;
import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.subsystems.Controllers.MichaelPID;
import org.firstinspires.ftc.teamcode.subsystems.GVF.GVFFollower;
import org.firstinspires.ftc.teamcode.subsystems.MathUtils.Pose2D;
import org.firstinspires.ftc.teamcode.subsystems.MathUtils.Vector;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Trajectory.Lerp;
import org.firstinspires.ftc.teamcode.subsystems.Trajectory.Trajectory;
import org.firstinspires.ftc.teamcode.subsystems.Trajectory.TrajectorySegment;
import org.firstinspires.ftc.teamcode.vision.TeamPropProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;

import java.util.ArrayList;
import java.util.Arrays;

@Autonomous
@Disabled
public class RedAudiencePixelDrop extends Robot {
    public static double correctionFactor = 0.2;
    public static double currentVelocityScalar = 0.01;
    public static double pathEndGain = 2;
    public static double curvatureGain = 0;
    /**
     * Increasing this number increases the area in which the curvature is accounted for.
     */
    public static double curvatureLookahead = 5;

    public static double minVelocity = 0;
    public static double maxVelocity = 1;
    public static double maxAcceleration = 0.5;
    public static int numWheels = 2;

    public static double intakePower = 0.4;

    public static double leftpos = 32;

    private VisionPortal visionportal;


    private GVFFollower pathfollower;
    public void runOpMode() {
        initialize();
        initGVF();
        TeamPropProcessor propProcessor = new TeamPropProcessor(true);
        initVisionPortal(new ArrayList<>(Arrays.asList(propProcessor)));
        slides.reset();

        swerve.setAuton();

        transferStates = states.INIT;

        ArrayList<Vector> controlright = new ArrayList<>(Arrays.asList(
                new Vector(0, 0),
                new Vector(20, -7)
        ));

        ArrayList<Vector> controlmid = new ArrayList<>(Arrays.asList(
                new Vector(0, 0),
                new Vector(25, 0)
        ));

        ArrayList<Vector> controlleft = new ArrayList<>(Arrays.asList(
                new Vector(0, 0),
                new Vector(28, 4)
        ));

        ArrayList<Vector> controlheadingleft = new ArrayList<>(Arrays.asList(
                new Vector(0, 0),
                new Vector(Math.PI/2, 0)
        ));

        ArrayList<Vector> controlheading = new ArrayList<>(Arrays.asList(
                new Vector(0, 0),
                new Vector(0, 0)
        ));

        ArrayList<Vector> controlend = new ArrayList<>(Arrays.asList(
                new Vector(0, 0),
                new Vector(Math.PI/2, 0)
        ));

        ArrayList<ArrayList<Vector>> controls = new ArrayList<>(Arrays.asList(controlend, controlheading, controlleft, controlheadingleft, controlmid, controlright));

        for (ArrayList<Vector> a : controls) {
            for (Vector v : a) {
                v = new Vector(v.get(0), -v.get(1));
            }
        }

        // right
        Trajectory trajectoryright = new Trajectory(new ArrayList<>(Arrays.asList(
                new TrajectorySegment(new Lerp(controlright), true))));

        // middle
        Trajectory trajectorymiddle = new Trajectory(new ArrayList<>(Arrays.asList(
                new TrajectorySegment(new Lerp(controlmid), true))));

        // left
        Trajectory trajectoryleft = new Trajectory(new ArrayList<>(Arrays.asList(
                new TrajectorySegment(new Lerp(controlleft), true))));

        Trajectory headingTrajectoryLeft = new Trajectory(new ArrayList<>(Arrays.asList(
                new TrajectorySegment(new Lerp(controlheadingleft))
        )));


        Trajectory headingTrajectory = new Trajectory(new ArrayList<>(Arrays.asList(
                new TrajectorySegment(new Lerp(controlheading))
        )));

        int position = -1;

        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);

        while(opModeInInit()) {
            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }
            try {
                position = propProcessor.getPosition();
                telemetry.addData("Position", position);
            } catch (Exception e) {}


            update();
            telemetry.addData("slidesPos", slides.getPosition());
            telemetry.addLine("Done initializing, press play to win against Oliver Buehrer (or not if slide no workie and >18 inch)");
            telemetry.update();
        }
        switch (position) {
            case TeamPropProcessor.LEFT:
                pathfollower.initTrajectory(trajectoryright, headingTrajectory, 0.25, 0.05, 100, new Pose2D(0, 0, 0));
                break;
            case TeamPropProcessor.MIDDLE:
                pathfollower.initTrajectory(trajectorymiddle, headingTrajectory, 0.25, 0.05, 100, new Pose2D(0, 0, 0));
                break;
            case TeamPropProcessor.RIGHT:
                pathfollower.initTrajectory(trajectoryleft, headingTrajectoryLeft, 0.25, 0.05, 100, new Pose2D(0, 0, 0));
                break;
        }

        odometry.reset(0, 0);
        resetIMU();
        pathfollower.start();
        transferStates = states.FOLDEDFAR;
        ElapsedTime timeout = new ElapsedTime();

        boolean hasinitedtraj = false;

        while (opModeIsActive()) {
            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }

            Pose2d odopose = odometry.getPose();
            Pose2D movepose = new Pose2D(0, 0, 0);

            Log.println(Log.INFO, "Odomoetry", "x: " + odopose.getX() + ", y: " + odopose.getY() + ", rot: " + odopose.getRotation().getDegrees());
            odometry.update(getHeading());

            if (!pathfollower.isStopped) {

                pathfollower.update(Pose2D.fromFTCLibPose(odometry.getPose()), getLoopSeconds());


                movepose = pathfollower.get();
            }


            if (!pathfollower.isStopped && timeout.seconds() < 5.0) {
                swerve.driveFieldCentric(-movepose.vals.get(1), movepose.vals.get(0), Range.clip(movepose.vals.get(2), -1, 1));
            } else if (timeout.seconds() < 7.0) {
                pathfollower.stop();
                intake.intake(-intakePower);
                swerve.driveRobotCentric(0, 0, 0);
                swerve.stop();
            } else {
                intake.stop();
            }



        }


    }

    private void initGVF() {
        pathfollower = new GVFFollower(correctionFactor, currentVelocityScalar, new MichaelPID(kHeadingP, kHeadingI, kHeadingD), pathEndGain, curvatureGain, curvatureLookahead);
        pathfollower.initRobot(minVelocity, maxVelocity, maxAcceleration, numWheels);
    }

    public void initVisionPortal(ArrayList<VisionProcessor> processors) {
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

        visionportal = builder.build();
    }
}
