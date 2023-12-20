package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Robot;


@Config
@TeleOp(group="Tuning")
public class ArmTuner extends Robot {

    public static int armExtensionTarget = 0;
    public static double armTiltTargetDegrees = 0;

    @Override
    public void runOpMode() {
        initialize();
        waitForStart();
        setFSMtoAuto();
        while (opModeIsActive()) {
            update();
            arm.extend(armExtensionTarget);
            arm.tiltArm(armTiltTargetDegrees);
            telemetry.addData("Arm Position", arm.getPosition());
            telemetry.addData("Arm Target", armExtensionTarget);
            telemetry.addData("Tilt Position", arm.getTilt());
            telemetry.addData("Tilt Target", armTiltTargetDegrees);
            telemetry.addData("Tilt Ticks", arm.getTiltTicks());
            telemetry.addData("Tilt Target Ticks", arm.getTiltTargetTicks());
        }
    }

}
