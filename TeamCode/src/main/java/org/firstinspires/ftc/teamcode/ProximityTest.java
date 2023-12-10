package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drivers.APDS9960;

@Disabled
@TeleOp
public class ProximityTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        ElapsedTime time = new ElapsedTime();
        APDS9960 apds9960 = hardwareMap.get(APDS9960.class, "color");
        DigitalChannel interrupt = hardwareMap.get(DigitalChannel.class, "interrupt");


        telemetry.addLine(apds9960.getDeviceName());
        while(opModeInInit()) {

        }
        while (opModeIsActive()) {
            telemetry.addData("proximity",apds9960.getProximity());
            telemetry.addData("time", time.milliseconds());
            telemetry.addData("interrupt", interrupt.getState());
            telemetry.addData("pint", apds9960.pint());
            time.reset();
            telemetry.update();

        }
    }
}
