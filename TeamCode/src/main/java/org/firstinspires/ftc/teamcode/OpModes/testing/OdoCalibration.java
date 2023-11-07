package org.firstinspires.ftc.teamcode.OpModes.testing;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Robot;
@TeleOp(group="aaa")
public class OdoCalibration extends Robot {
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        while (opModeIsActive()) {
            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }
            odometry.update(Robot.getHeading());


            telemetry.update();
        }
    }
}
