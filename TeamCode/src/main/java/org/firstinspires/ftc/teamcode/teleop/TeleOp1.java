package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@TeleOp(name="TestTeleOp")
public class TeleOp1 extends Robot {
    ElapsedTime time = new ElapsedTime();

    @Override
    public void runOpMode() {
        initialize();
        waitForStart();
        while(opModeIsActive()) {
            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }



            //swerve.goofyDrive(AngleUnit.normalizeDegrees(Math.toDegrees(Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x))+90), -gamepad1.right_stick_y);
            swerve.driveFieldCentric(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);
            if (gamepad1.options) {
                resetIMU();
            }
            if (gamepad1.right_trigger>0) {
                intake.intake(gamepad1.right_trigger);
            }
            else if (gamepad1.left_trigger>0) {
                intake.intake(-gamepad1.left_trigger);
            }
            else
                intake.stop();
            if (gamepad1.a) {
                telemetry.addData("funny", "its not actually funny");
            }
            telemetry.addData("time",time.milliseconds());
            time.reset();

        }
    }
}
