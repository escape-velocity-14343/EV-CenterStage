package org.firstinspires.ftc.teamcode.OpModes.auton;

import static org.firstinspires.ftc.teamcode.OpModes.auton.AutonBase.Alliance.BLUE;
import static org.firstinspires.ftc.teamcode.OpModes.auton.AutonBase.Alliance.RED;

import android.util.Log;
import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
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

@Config
public abstract class AutonBase extends Robot {
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
    Alliance alliance = RED;
    Side side = Side.BACKDROP;


    private GVFFollower pathfollower;

    enum Side {
        AUDIENCE,
        BACKDROP
    }
    enum Alliance {
        BLUE,
        RED
    }
    public void run() {
        initialize();
        initGVF();
        TeamPropProcessor propProcessor = new TeamPropProcessor(true);
        initVisionPortal(new ArrayList<>(Arrays.asList(propProcessor)));
        slides.reset();

        swerve.setAuton();

        transferStates = states.INIT;

        ArrayList<Vector> controlright = new ArrayList<>(Arrays.asList(
                new Vector(0, 0),
                new Vector(17, -7)
        ));

        ArrayList<Vector> controlmid = new ArrayList<>(Arrays.asList(
                new Vector(0, 0),
                new Vector(20, 0)
        ));

        ArrayList<Vector> controlleft = new ArrayList<>(Arrays.asList(
                new Vector(0, 0),
                new Vector(22, 0)
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
        ArrayList<Vector> controlheadingright = new ArrayList<>(Arrays.asList(
                new Vector(0, 0),
                new Vector(0, 0)
        ));
        ArrayList<ArrayList<Vector>> controls = new ArrayList<>(Arrays.asList(controlleft, controlmid, controlright));
        ArrayList<ArrayList<Vector>> headingcontrols = new ArrayList<>(Arrays.asList(controlend, controlheading, controlheadingleft, controlheadingright));



        int position = -1;

        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);


        while(opModeInInit()) {
            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }
            try {
                position = propProcessor.getPosition();
                double[] avgs = propProcessor.getVals();
                switch(position) {
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


            update();
            telemetry.addData("slidesPos", slides.getPosition());
            telemetry.addLine("Initialized, dont press start please");
            if (gamepad1.dpad_up) {
                alliance = BLUE;
                propProcessor.setTeam(false);
            }
            if (gamepad1.dpad_down) {
                alliance = RED;
                propProcessor.setTeam(true);
            }
            if (gamepad1.dpad_left) {
                side = Side.AUDIENCE;
            }
            if (gamepad1.dpad_right) {
                side = Side.BACKDROP;
            }
            telemetry.addData("side", side);
            telemetry.addData("alliance", alliance);
            telemetry.update();
        }
        if (side==Side.AUDIENCE) {
            position = 2-position;
        }
        if(alliance==Alliance.BLUE) {
            position = 2-position;
        }
        for (ArrayList<Vector> a : controls) {
            for (int i = 0; i < a.size(); i++) {
                a.set(i, new Vector(a.get(i).get(0), a.get(i).get(1)*(side==Side.BACKDROP ? 1: -1)*(alliance==Alliance.RED ? 1: -1)));
            }
        }

        for (ArrayList<Vector> a : headingcontrols) {
            for (int i = 0; i < a.size(); i++) {
                a.set(i, new Vector(a.get(i).get(0)*(side==Side.BACKDROP ? 1: -1)*(alliance==Alliance.RED ? 1: -1), a.get(i).get(1)));
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
        Trajectory headingTrajectoryRight = new Trajectory(new ArrayList<>(Arrays.asList(
                new TrajectorySegment(new Lerp(controlheadingright))
        )));

        Trajectory secondHeadingTraj = new Trajectory(new ArrayList<>(Arrays.asList(
                new TrajectorySegment(new Lerp(controlend))
        )));
        switch (position) {
            case TeamPropProcessor.RIGHT:
                pathfollower.initTrajectory(trajectoryright, headingTrajectoryRight, 0.25, 0.05, 100, new Pose2D(0, 0, 0));
                break;
            case TeamPropProcessor.MIDDLE:
                pathfollower.initTrajectory(trajectorymiddle, headingTrajectory, 0.25, 0.05, 100, new Pose2D(0, 0, 0));
                break;
            case TeamPropProcessor.LEFT:
                pathfollower.initTrajectory(trajectoryleft, headingTrajectoryLeft, 0.25, 0.05, 100, new Pose2D(0, 0, 0));
                break;
        }

        odometry.reset(0, 3*(side==Side.BACKDROP?1:-1)*(alliance==RED?1:-1));
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

            } else if (timeout.seconds() < 15.0 && !hasinitedtraj) {
                if (side == Side.AUDIENCE) {
                    break;
                }
                Trajectory secondtraj = new Trajectory(new ArrayList<>(Arrays.asList(
                           new TrajectorySegment(new Lerp(new ArrayList<>(Arrays.asList(
                                new Vector(odopose.getX(), odopose.getY()),
                                new Vector((22 + (position==0?5:position==1?0:-5)), -leftpos*(alliance==RED?1:-1))
                        ))), true))));
                pathfollower.initTrajectory(secondtraj, secondHeadingTraj, 0.25, 0.05, 100, Pose2D.fromFTCLibPose(odopose));
                pathfollower.start();
                hasinitedtraj = true;
            } else if (timeout.seconds() < 15.0) {
                intake.stop();
                swerve.setAuton();
                swerve.driveFieldCentric(-movepose.vals.get(1), movepose.vals.get(0), Range.clip(movepose.vals.get(2), -1, 1));
            } else {
                swerve.driveRobotCentric(0, 0, 0);
                transferStates = states.OUTTAKE;
                intake.stop();
                if (timeout.seconds() > 20.0) {
                    bucket.unLatch();
                }
            }

            update();



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