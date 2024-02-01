package org.firstinspires.ftc.teamcode.vision;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.vision.VisionPortal;

@TeleOp
public class StackDetectionTest extends Robot {
    private VisionPortal visionportal;
    private StackLineDetectionProcessor stackdetect = new StackLineDetectionProcessor();
    @Override
    public void runOpMode() {
        //stackdetect.setAngle(60);

        initPortal();
        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("aosidjfaosdjfoij",stackdetect.getMiddle());
        }
    }


    public void initPortal() {
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        builder.setCamera(hardwareMap.get(WebcamName.class, "bucket camera"));

        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(new Size(320, 240));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        builder.setAutoStopLiveView(false);

        builder.addProcessor(stackdetect);
        visionportal = builder.build();
        visionportal.setProcessorEnabled(stackdetect, true);

    }

}