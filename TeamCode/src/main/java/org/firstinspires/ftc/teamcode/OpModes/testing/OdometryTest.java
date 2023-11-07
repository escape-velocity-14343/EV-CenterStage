package org.firstinspires.ftc.teamcode.OpModes.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Robot;
@TeleOp(group="aaa")
public class OdometryTest extends Robot {
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        swerve.headingLock(true,0);
        while (opModeIsActive()) {
            telemetryPacket = new TelemetryPacket();
            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }
            odometry.update(Robot.getHeading());

            if (gamepad1.triangle) {
                odometry.reset();
            }
            if (gamepad1.square) {
                swerve.headingLock(true,Math.PI/2);
            }
            swerve.driveFieldCentric(0,0,0);
            telemetry.update();
        }

    }
}
