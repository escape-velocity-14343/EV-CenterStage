package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class SwerveMotorTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        Motor lBottom = new Motor(hardwareMap,"bottomleft");
        Motor lTop = new Motor(hardwareMap,"topleft");
        Motor rBottom = new Motor(hardwareMap,"bottomright");
        Motor rTop = new Motor(hardwareMap,"topright");
        waitForStart();
        while (opModeIsActive()) {
            lBottom.set(gamepad1.left_stick_x);
            lTop.set(gamepad1.left_stick_y);
            rBottom.set(gamepad1.right_stick_x);
            rTop.set(gamepad1.right_stick_y);
        }

    }
}
