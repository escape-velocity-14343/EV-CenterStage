package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pathutils.AutonomousWaypoint;
import org.firstinspires.ftc.teamcode.subsystems.ArmIVK;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@Config
@TeleOp
public class ArmHoldFieldPositionTest extends Robot {
    public static double targetx = -10;
    public static double targetheight = 5;
    public static double targetbucketangle = Math.PI;
    public static boolean test = true;

    @Override
    public void runOpMode() {
        initialize();
        while (opModeInInit()) {
            telemetry.addData("Slide extension", arm.getPosition());
            telemetry.addData("Target slide extension", ArmIVK.getSlideExtension());
            telemetry.addData("Target slide tilt", ArmIVK.getArmAngle());
            telemetry.addData("Slide tilt", arm.getTilt());
            telemetry.update();
        }
        AutonomousWaypoint.configAuto(true, true);
        double distance = -20;
        double height = targetheight;
        ElapsedTime thing = new ElapsedTime();
        while (opModeIsActive()) {
            update();
            if (!test) {
                ArmIVK.calcIVK(AutonomousWaypoint.distance(odometry.getPose(), new AutonomousWaypoint(targetx, 0, 0)), targetheight, targetbucketangle);
                forceGoToArmIVK();
            } else {
                if (thing.seconds() < 4) {
                    distance = -20 - thing.seconds() * 10;
                } else if (thing.seconds() > 8&&thing.seconds() < 12) {
                    distance = -60 + (thing.seconds()-8) * 10;
                }
                ArmIVK.calcIVK(distance, height, targetbucketangle);
                forceGoToArmIVK();
            }
            telemetry.addData("Slide extension", arm.getPosition());
            telemetry.addData("Target slide extension", ArmIVK.getSlideExtension());
            telemetry.addData("Target slide tilt", ArmIVK.getArmAngle());
            telemetry.addData("Slide tilt", arm.getTilt());
        }
    }
}
