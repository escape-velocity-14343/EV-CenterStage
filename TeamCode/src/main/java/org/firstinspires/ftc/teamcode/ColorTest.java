package org.firstinspires.ftc.teamcode;





import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drivers.APDS9960;
@Config
@Disabled
@TeleOp
public class ColorTest extends LinearOpMode {
    public static int wait = 10;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
        APDS9960 apds9960 = hardwareMap.get(APDS9960.class, "color");
        ElapsedTime time = new ElapsedTime();

        telemetry.addLine(apds9960.getDeviceName());
        apds9960.setATime(wait);
        LynxModule module = hardwareMap.getAll(LynxModule.class).get(0);
        module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);


        while(opModeInInit()) {

        }
        while (opModeIsActive()) {
            NormalizedRGBA color = apds9960.getColor();
            int rgb = (((int) color.red)<<16)|(((int) color.green)<<8)|((int) color.blue&0x0ff);
            module.setConstant(rgb);


            telemetry.addData("r",color.red);
            telemetry.addData("g",color.green);
            telemetry.addData("b",color.blue);
            telemetry.addData("time", time.milliseconds());
            time.reset();
            telemetry.update();

        }

    }
}
