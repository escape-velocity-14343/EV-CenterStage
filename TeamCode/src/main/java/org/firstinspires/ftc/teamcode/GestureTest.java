package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drivers.APDS9960;

@Disabled
@TeleOp
public class GestureTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        APDS9960 apds9960 = hardwareMap.get(APDS9960.class, "color");
        ElapsedTime time = new ElapsedTime();

        telemetry.addLine(apds9960.getDeviceName());


        while(opModeInInit()) {

        }
        while (opModeIsActive()) {
            telemetry.addData("up",apds9960.getUp());

            telemetry.addData("time", time.milliseconds());
            time.reset();
            telemetry.update();

        }
    }
}
