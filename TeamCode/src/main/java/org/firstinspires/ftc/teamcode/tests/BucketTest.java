package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Bucket;

@TeleOp(group="Tests")
public class BucketTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        Bucket bucket = new Bucket(hardwareMap);
        boolean isDropping = false;
        waitForStart();
        while (opModeIsActive()) {
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
