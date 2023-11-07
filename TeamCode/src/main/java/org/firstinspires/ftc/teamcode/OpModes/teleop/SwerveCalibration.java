package org.firstinspires.ftc.teamcode.OpModes.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Robot;
@TeleOp (group = "aaa")
@Config
public class SwerveCalibration extends Robot {
    public static double pidPosition=0;
    public static double move = 1;
    @Override
    public void runOpMode() {
        initialize();
        waitForStart();
        while (opModeIsActive()) {
            if (move!=0)
                swerve.left.podPid(0,pidPosition);
            else
                swerve.left.podMove(0,0);
        telemetry.update();
        }
    }
}
