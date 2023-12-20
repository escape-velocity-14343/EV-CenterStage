package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Arm;

@TeleOp(group = "Tests")
public class ArmTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        Arm arm = new Arm(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.left_trigger > 0.7) {
                arm.moveSlides(gamepad1.left_trigger);
            } else if (gamepad1.right_trigger > 0.7) {
                arm.moveSlides(-gamepad1.right_trigger);
            }
            if (gamepad1.left_bumper) {
                arm.moveTilt(1);
            } else if (gamepad1.right_bumper) {
                arm.moveTilt(-1);
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
