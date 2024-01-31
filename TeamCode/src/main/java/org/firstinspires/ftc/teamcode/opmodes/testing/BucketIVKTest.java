package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.ArmIVK;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@Config
@TeleOp(group="test")
public class BucketIVKTest extends Robot {

    public static double targetBucketAngle = 0;

    @Override
    public void runOpMode() {
        initialize();
        waitForStart();
        swerve.setAuton();
        setFSMtoAuto();
        while (opModeIsActive()) {
            if (Math.abs(gamepad1.right_stick_x) > 0.1) {
                arm.moveTilt(gamepad1.right_stick_x);
            } else {
                arm.moveTilt(0);
            }

            bucket.tilt(0, ArmIVK.getBucketTilt(Math.toRadians(arm.getTilt()), targetBucketAngle));
            telemetry.addData("bucket thing", ArmIVK.getBucketTilt(Math.toRadians(arm.getTilt()), targetBucketAngle));
            update();
        }
    }
}
