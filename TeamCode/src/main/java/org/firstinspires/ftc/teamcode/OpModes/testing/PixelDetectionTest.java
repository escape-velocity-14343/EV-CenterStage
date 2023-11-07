package org.firstinspires.ftc.teamcode.OpModes.testing;





import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drivers.APDS9960;
import org.firstinspires.ftc.teamcode.subsystems.Bucket;
import org.firstinspires.ftc.teamcode.vision.ColorPixelDetection;

@Config
@TeleOp(group="aaaaa")
public class PixelDetectionTest extends LinearOpMode {
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

        Bucket bucket = new Bucket(hardwareMap);


        while(opModeInInit()) {

        }
        while (opModeIsActive()) {
            ColorPixelDetection[] detections = bucket.getPixelColors();
            for (int i = 0; i < detections.length; i++) {
                telemetry.addData("Detection", detections[i].type);
            }
            telemetry.update();

        }

    }
}
