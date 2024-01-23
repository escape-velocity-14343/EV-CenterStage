package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Robot;

@Config
@TeleOp
public class UltrasonicTest extends Robot {
    public static boolean runArm = false;
    public static double ultrasonicTarget = 10;
    @Override
    public void runOpMode() {
        initialize();
        waitForStart();
        setFSMtoAuto();
        swerve.setAuton();
        while (opModeIsActive()) {
            update();
            telemetry.addData("ultrasonic dist", arm.getUltrasonicInches());
            telemetry.addData("ultrasonic target", ultrasonicTarget);
            if (runArm) {
                arm.ultrasonicExtend(ultrasonicTarget);
            }

        }
    }
}
