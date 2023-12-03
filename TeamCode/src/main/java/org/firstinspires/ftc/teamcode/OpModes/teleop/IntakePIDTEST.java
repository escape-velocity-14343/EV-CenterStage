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

public class IntakePIDTEST extends Robot {
    public static int intakePos = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        //initialize();
        intake = new Intake(hardwareMap);
        allHubs = hardwareMap.getAll(LynxModule.class);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        waitForStart();
        while (!isStopRequested()) {
            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }
            intake.pidIntake(intakePos);
            telemetry.addData("Intake position", intake.getPosition());
            telemetry.addData("Target position", intakePos);
            telemetry.update();
        }
    }
}
