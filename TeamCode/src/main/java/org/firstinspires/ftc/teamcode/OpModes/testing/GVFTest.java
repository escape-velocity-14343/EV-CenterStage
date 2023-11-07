package org.firstinspires.ftc.teamcode.OpModes.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.subsystems.Controllers.MichaelPID;
import org.firstinspires.ftc.teamcode.subsystems.GVF.GVFFollower;
import org.firstinspires.ftc.teamcode.subsystems.MathUtils.Pose2D;
import org.firstinspires.ftc.teamcode.subsystems.MathUtils.Vector;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Trajectory.CubicBezier;
import org.firstinspires.ftc.teamcode.subsystems.Trajectory.Lerp;
import org.firstinspires.ftc.teamcode.subsystems.Trajectory.Trajectory;
import org.firstinspires.ftc.teamcode.subsystems.Trajectory.TrajectorySegment;

import java.util.ArrayList;
import java.util.Arrays;

@Config
@Autonomous
public class GVFTest extends Robot {

    public static double correctionFactor = 0.1;
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


    private GVFFollower pathfollower;
    public void runOpMode() {
        initialize();
        initGVF();

        Trajectory trajectory = new Trajectory(new ArrayList<>(Arrays.asList(
                new TrajectorySegment(new CubicBezier(new ArrayList<>(Arrays.asList(
                        new Vector(0, 0),
                        new Vector(3.33, 5),
                        new Vector(6.66, 10),
                        new Vector(10, 0)
                ))), true)
        )));

        Trajectory headingTrajectory = new Trajectory(new ArrayList<>(Arrays.asList(
                new TrajectorySegment(new Lerp(new ArrayList<>(Arrays.asList(
                        new Vector(0, 0),
                        new Vector(0, 0)
                ))))
        )));

        pathfollower.initTrajectory(trajectory, headingTrajectory, 2, 100, 100, new Pose2D(0, 0, 0));

        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);

        while(opModeInInit()) {

        }

        pathfollower.start();

        while (opModeIsActive()) {
            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }
            odometry.update(getHeading());

            pathfollower.update(Pose2D.fromFTCLibPose(odometry.getPose()), getLoopSeconds());


            Pose2D movepose = pathfollower.get();


            swerve.driveFieldCentric(-movepose.vals.get(1), movepose.vals.get(0), Range.clip(movepose.vals.get(2), -1, 1));

        }


    }

    private void initGVF() {
        pathfollower = new GVFFollower(correctionFactor, currentVelocityScalar, new MichaelPID(kHeadingP, kHeadingI, kHeadingD), pathEndGain, curvatureGain, curvatureLookahead);
        pathfollower.initRobot(minVelocity, maxVelocity, maxAcceleration, numWheels);
    }
}
