package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pathutils.AutonomousWaypoint;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@Config
@TeleOp(group="testing")
public class PosSquIDTest extends Robot {

    public static double PIDpos = 0;
    public static double pidy = 0;

    public static boolean isRed = true;
    public static boolean isBackstage = true;
    public static double audienceoffset = 0;
    public static boolean turntopoint = false;

    @Override
    public void runOpMode() {
        initialize();
        waitForStart();
        swerve.setAuton();

        while (opModeIsActive()) {
            update();
            AutonomousWaypoint.configAuto(isRed, isBackstage);
            if (!turntopoint) {
                goToPoint(new AutonomousWaypoint(PIDpos, pidy, Math.toRadians(90))
                        .setAudienceOffset(audienceoffset, 0, 0)
                        .setBlueHeadingReversed());
                telemetry.addData("target", PIDpos);
                telemetry.addData("pos", odometry.getPose().getX());
            } else {
                goToPoint(new AutonomousWaypoint(odometry.getPose().getX(), odometry.getPose().getY(), new AutonomousWaypoint(0, 0, 0)));
            }
        }
    }
}
