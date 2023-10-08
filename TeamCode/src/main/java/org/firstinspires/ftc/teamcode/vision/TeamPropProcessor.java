package org.firstinspires.ftc.teamcode.vision;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

public class TeamPropProcessor implements VisionProcessor {

    /**
     * Use TeamPropProcessor.RED or TeamPropProcessor.BLUE.
     * Values: RED = true, BLUE = false.
     */
    boolean team;

    /**
     * For team indication.
     */
    public static boolean RED = true;

    /**
     * For team indication.
     */
    public static boolean BLUE = false;

    int camwidth;
    int camheight;

    private Mat YCrCb = new Mat();
    private Rect leftrect;
    private Rect midrect;
    private Rect rightrect;

    private Integer placement;

    public TeamPropProcessor(boolean team) {
        this.team = team;
    }

    @Override
    public void init(int width, int height, CameraCalibration calib) {
        camwidth = width;
        camheight = height;

        int height_upper = (int)((0.333)*camheight);
        int height_lower = (int)((0.666)*camheight);
        int width_left = (int)((0.333)*camwidth);
        int width_right = (int)((0.666)*camwidth);

        leftrect = new Rect(0, height_upper, width_left, height_lower-height_upper);
        midrect = new Rect(width_left, height_upper, width_right-width_left, height_lower-height_upper);
        rightrect = new Rect(width_right, height_upper, camwidth-width_right, height_lower-height_upper);
    }

    @Override
    /**
     *
     */
    public Object processFrame(Mat input, long captureTimeNanos) {
        Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);


        Mat leftmat = input.submat(leftrect);
        Mat midmat = input.submat(midrect);
        Mat rightmat = input.submat(rightrect);

        int index;
        if (team) {
            index = 1;
        } else {
            index = 2;
        }

        double leftavg = Core.mean(leftmat).val[index];
        double midavg = Core.mean(midmat).val[index];
        double rightavg = Core.mean(rightmat).val[index];

        if (leftavg > midavg && leftavg > rightavg) {
            placement = 0;
            return (Integer) 0;
        } else if (midavg > leftavg && midavg > rightavg) {
            placement = 1;
            return (Integer) 1;
        } else {
            placement = 2;
            return (Integer) 2;
        }


    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        // dont do anything lmao
        return;
    }

    /**
     * 0: Left.
     * 1: Middle.
     * 2: Right.
     */
    public Integer getPosition() {
        return placement;
    }


}
