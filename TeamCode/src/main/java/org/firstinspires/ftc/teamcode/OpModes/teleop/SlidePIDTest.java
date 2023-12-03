package org.firstinspires.ftc.teamcode.OpModes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
@TeleOp
@Config

public class SlidePIDTest extends Robot {
    public static int slidePos = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        while (!isStopRequested()) {
            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }
            slides.pidSlides(slidePos);
            telemetry.addData("Slide position", slides.getPosition());
            telemetry.addData("Target position", slidePos);
            telemetry.update();
        }
    }
}
