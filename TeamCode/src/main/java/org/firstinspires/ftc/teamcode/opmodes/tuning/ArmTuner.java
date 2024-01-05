package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.ArmIVK;
import org.firstinspires.ftc.teamcode.subsystems.Robot;


@Config
@TeleOp(group="Tuning")
public class ArmTuner extends Robot {

    public static int armExtensionTarget = 0;
    public static double armTiltTargetDegrees = 0;
    public static double bucketTiltDegrees = 0;
    public static boolean useArmIVK = false;
    public static double armIVKDistance = 0;
    public static double armIVKHeight = 0;

    @Override
    public void runOpMode() {
        initialize();
        waitForStart();
        setFSMtoAuto();
        while (opModeIsActive()) {
            if (useArmIVK) {
                ArmIVK.calcBackdropIVK(armIVKDistance, armIVKHeight);
                goToArmIVK();
                telemetry.addData("Bucket target", ArmIVK.getBucketTilt());
                telemetry.addData("Arm Angle target", ArmIVK.getArmAngle());
                telemetry.addData("Arm Extension target", ArmIVK.getSlideExtension());
            } else {
                arm.extendInches(armExtensionTarget);
                arm.tiltArm(armTiltTargetDegrees);
                bucket.tilt(ArmIVK.getBucketTilt(Math.toRadians(arm.getTilt()), Math.toRadians(bucketTiltDegrees)));
            }
            telemetry.addData("Arm Position", arm.getPosition());
            telemetry.addData("Arm Target", armExtensionTarget);
            telemetry.addData("Tilt Position", arm.getTilt());
            telemetry.addData("Tilt Target", armTiltTargetDegrees);
            update();
        }
    }

}
