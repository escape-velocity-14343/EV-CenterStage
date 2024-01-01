package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Robot;

@Config
@TeleOp(group="test")
public class OdoTuner extends Robot {

    public static double x = 0;
    public static double y = 0;
    public static double heading = 0;
    @Override
    public void runOpMode() {
        initialize();

        while (opModeInInit()) {
            setPoseEstimate(new Pose2d(x, y, new Rotation2d(Math.toRadians(heading))));
        }


        swerve.setAuton();
        while (opModeIsActive()) {
            update();
            swerve.driveFieldCentric(-gamepad1c.left_stick_x, -gamepad1c.left_stick_y, gamepad1c.right_stick_x);
            int[] ticks = odometry.getTicks();
            telemetry.addData("Odometry x ticks", ticks[0]);
            telemetry.addData("Odometry y ticks", ticks[1]);

        }
    }
}
