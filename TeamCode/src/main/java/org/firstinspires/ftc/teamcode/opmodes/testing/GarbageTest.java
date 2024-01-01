package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Bucket;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.SwerveController;

@TeleOp(group="test")
public class GarbageTest extends Robot {

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        while(!isStopRequested()) {
            update();
            swerve.driveFieldCentric(-gamepad1c.left_stick_x, -gamepad1c.left_stick_y, gamepad1c.right_stick_x);
            telemetry.addData("left rotation",swerve.getRotations()[0]);
            telemetry.addData("right rotation", swerve.getRotations()[1]);
        }
        Arm arm = new Arm(hardwareMap);
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
        Bucket bucket = new Bucket(hardwareMap);
        boolean isDropping = false;
        while (opModeIsActive()) {
            bucket.tilt(gamepad1.left_stick_y);
            bucket.update();
            if (gamepad1.dpad_left) {
                bucket.setLeftLatch(false);
            } else if (gamepad1.dpad_right) {
                bucket.setRightLatch(false);
            } else if (gamepad1.dpad_down) {
                bucket.unlatch();
            } else if (gamepad1.dpad_up) {
                bucket.dropFromStack();
            } else {
                bucket.smartLatch();
            }
            telemetry.addData("Number of Pixels", bucket.getNumPixels());
            telemetry.update();
        }
    }
}
