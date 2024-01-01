package org.firstinspires.ftc.teamcode.opmodes.testing;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Robot;
@Config
@TeleOp(group="Testing")
public class ArmRotationTest extends Robot {
    public static double rotationOffset = 0;
    public static boolean invert = false;

    @Override
    public void runOpMode() {
        initialize();
        waitForStart();
        while (opModeIsActive()) {
            update();
            arm.setArmOffset(rotationOffset);
            arm.setSensorInverted(invert);
            telemetry.addData("Tilt Position",arm.getTilt());

        }

    }

}
