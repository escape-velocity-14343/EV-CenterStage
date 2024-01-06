package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Arm;

@TeleOp(group = "Tests")
public class ArmTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        Arm arm = new Arm(hardwareMap,telemetry);
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.left_trigger > 0.1) {
                arm.moveSlides(gamepad1.left_trigger);
            } else if (gamepad1.right_trigger > 0.1) {
                arm.moveSlides(-gamepad1.right_trigger);
            } else {
                arm.moveSlides(0);
            }

            if (Math.abs(gamepad1.right_stick_x) > 0.1) {
                arm.moveTilt(gamepad1.right_stick_x);
            } else {
                arm.moveTilt(0);
            }
            if (gamepad1.dpad_up) {
                arm.holdPosition();
            }
            if (gamepad1.dpad_down) {
                arm.extend(1000);
            }
            if (gamepad1.dpad_left) {
                arm.tiltArm(0);
            }
        }
    }
}
