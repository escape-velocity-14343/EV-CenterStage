package org.firstinspires.ftc.teamcode.OpModes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Robot;
@TeleOp(group = "asidifj")
public class FlipIntakeTest extends Robot {
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        while (!isStopRequested()) {
            if (gamepad1.triangle) {
                intake.armUp();
            }
            if (gamepad1.square) {
                intake.armDown();
            }
        }
    }
}
