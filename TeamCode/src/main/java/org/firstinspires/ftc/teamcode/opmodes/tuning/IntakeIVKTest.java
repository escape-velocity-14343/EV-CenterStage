package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.ArmIVK;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
@TeleOp
@Config


public class IntakeIVKTest extends Robot {
    public static double height = 0;
    public static double extension = 10;
    public static double servoVal = 0.5;
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        while (opModeInInit()) {
            bucket.tilt(servoVal, 0.5);
        }
        setFSMtoAuto();
        while (opModeIsActive()) {
            update();
            ArmIVK.calcIntakeIVK(extension, height, Math.toRadians(arm.getTilt()));
            bucket.tilt(ArmIVK.getBarTilt(),ArmIVK.getBucketTilt());
            telemetry.addData("bar tilt", ArmIVK.getBarTilt());
            telemetry.addData("bucket tilt", ArmIVK.getBucketTilt());
            //arm.extend(ArmIVK.getSlideExtension());
        }
    }
}
