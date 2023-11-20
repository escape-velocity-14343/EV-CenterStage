package org.firstinspires.ftc.teamcode.OpModes.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Robot;
@TeleOp
@Config

public class IntakePIDTEST extends Robot {
    public static int intakePos = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        while (!isStopRequested()) {
            intake.pidIntake(intakePos);
        }
    }
}
