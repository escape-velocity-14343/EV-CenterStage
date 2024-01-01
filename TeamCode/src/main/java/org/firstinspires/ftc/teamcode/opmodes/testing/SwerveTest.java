package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.SwerveController;

@Config
@TeleOp(group="test")
public class SwerveTest extends Robot {

    public static boolean drive = true;
    public static double leftTarget = 0;
    public static double rightTarget = 0;
    int cycles = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        swerve.setAuton();
        waitForStart();
        ElapsedTime timer = new ElapsedTime();
        while(!isStopRequested()) {
            update();
            if (drive) {
                swerve.driveFieldCentric(-gamepad1c.left_stick_x, -gamepad1c.left_stick_y, gamepad1c.right_stick_x);
            } else {
                swerve.left.podPid(0, leftTarget);
                swerve.right.podPid(0, rightTarget);
                telemetry.addData("left target", leftTarget);
                telemetry.addData("right target", rightTarget);
            }
            telemetry.addData("left rotation",swerve.getRotations()[0]);
            telemetry.addData("right rotation", swerve.getRotations()[1]);
            cycles++;
            telemetry.addData("hz", cycles/timer.seconds());
        }

    }
}
