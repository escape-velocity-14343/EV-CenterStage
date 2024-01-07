package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class LiftTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        DcMotor hang = hardwareMap.dcMotor.get("intake");
        waitForStart();
        while (opModeIsActive()) {
            hang.setPower(gamepad1.right_stick_x);
        }
    }
}
