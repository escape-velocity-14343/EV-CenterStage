package org.firstinspires.ftc.teamcode.OpModes.testing;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drivers.APDS9960;
@Config
@TeleOp(group="aaaaa")
public class InterruptTest extends LinearOpMode {

    public static int val = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
        ElapsedTime time = new ElapsedTime();
        APDS9960 apds9960 = hardwareMap.get(APDS9960.class, "color");
        DigitalChannel interrupt = hardwareMap.get(DigitalChannel.class, "interrupt");


        telemetry.addLine(apds9960.getDeviceName());

        while(opModeInInit()) {

        }
        while (opModeIsActive()) {
            if (val!=0)
                apds9960.forceInt(val);
            telemetry.addData("proximity",apds9960.getProximity());
            telemetry.addData("time", time.milliseconds());
            telemetry.addData("interrupt", interrupt.getState());
            telemetry.addData("pint", Integer.toBinaryString(apds9960.pint()));
            time.reset();
            telemetry.update();

        }
    }
}
