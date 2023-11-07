package org.firstinspires.ftc.teamcode.vision;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;

import java.util.ArrayList;
import java.util.Arrays;

@Config
@Autonomous
public class PropTest extends Robot {

    private VisionPortal visionportal;
    private TeamPropProcessor propProcessor;


    public void runOpMode() {
        propProcessor = new TeamPropProcessor(TeamPropProcessor.RED);
        initVisionPortal(new ArrayList<VisionProcessor>(Arrays.asList(propProcessor)));

        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);

        while(opModeInInit()) {

        }
        while (opModeIsActive()) {
            try {
                switch (propProcessor.getPosition()) {
                    case TeamPropProcessor.LEFT:
                        telemetry.addLine("Position: Left");
                        break;
                    case TeamPropProcessor.MIDDLE:
                        telemetry.addLine("Position: Middle");
                        break;
                    case TeamPropProcessor.RIGHT:
                        telemetry.addLine("Position: Right");
                        break;
                }
            } catch (Exception e) {}


            // Push telemetry to the Driver Station.
            telemetry.update();

            sleep(20);

        }
    }

    public void initVisionPortal(ArrayList<VisionProcessor> processors) {
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        builder.setCamera(hardwareMap.get(WebcamName.class, "camera"));

        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        builder.setAutoStopLiveView(false);

        for (VisionProcessor processor : processors) {
            builder.addProcessor(processor);
        }

        visionportal = builder.build();
    }
}
