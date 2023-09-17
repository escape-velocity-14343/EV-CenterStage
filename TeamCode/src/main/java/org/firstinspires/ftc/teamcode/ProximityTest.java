package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class ProximityTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        APDS9960 apds9960 = hardwareMap.get(APDS9960.class, "color");
        telemetry.addLine(apds9960.getDeviceName());
        while(opModeInInit()) {

        }
        while (opModeIsActive()) {
            telemetry.addData("proximity",apds9960.getProximity());
            telemetry.update();
            sleep(100);
        }
    }
}
