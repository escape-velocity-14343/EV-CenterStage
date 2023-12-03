package org.firstinspires.ftc.teamcode.vision;

import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.util.Log;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.subsystems.MathUtils.Angle;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;


@Config
public class StackDetectionProcessor implements VisionProcessor {

    public static double contourAreaMultiplier = 0.05;
    public static int minsatval = 0;
    public static int minvalval = 170;
    public static int maxsatval = 40;
    public static boolean project = false;
    int totalArea;
    double totalAngle;
    double angleOut = 0;
    boolean last = false;
    Mat HSV = new Mat();
    Mat thresh = new Mat();
    Bitmap copy;

    @Override
    public void init(int width, int height, CameraCalibration calib) {
        totalArea = width*height;
    }

    /**
     * Set the total angle of the camera (total angle it can see).
     * @param angle In Radians.
     */
    public void setAngle(double angle) {
        this.totalAngle = angle;
    }

    @Override
    public Object processFrame(Mat input, long captureTimeNanos) {
        Imgproc.cvtColor(input, HSV, Imgproc.COLOR_RGB2HSV);

        Core.inRange(HSV, new Scalar(0, minsatval, minvalval), new Scalar(255, maxsatval, 255), thresh);

        ArrayList<MatOfPoint> contours = new ArrayList<>();

        Imgproc.findContours(thresh, contours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_NONE);
        Imgproc.cvtColor(HSV, HSV, Imgproc.COLOR_HSV2RGB);

        ArrayList<MatOfPoint> bigcontours = new ArrayList<>();
        MatOfPoint biggest = new MatOfPoint();
        double maxarea = 0;

        for (MatOfPoint contour : contours) {
            if (Imgproc.contourArea(contour) > totalArea*contourAreaMultiplier) {
                bigcontours.add(contour);
                if (Imgproc.contourArea(contour) > maxarea) {
                    biggest = contour;
                    maxarea = Imgproc.contourArea(contour);
                }
            }
        }



        if (bigcontours.size() > 0) {
            angleOut = (Imgproc.boundingRect(biggest).x+Imgproc.boundingRect(biggest).width/2.0)*totalAngle/input.width();
            last = true;
        } else {
            last = false;
        }

        Imgproc.drawContours(HSV, bigcontours, -1, new Scalar(0, 255, 0), 3);

        HSV.copyTo(input);


        copy = Bitmap.createBitmap(input.width(), input.height(), Bitmap.Config.RGB_565);
        Utils.matToBitmap(input, copy);


        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        //if (project) {
        canvas = new Canvas(copy);
        //}
    }

    /**
     * @return Returns the angle of the largest contour in radians, or 14343 if no such viable contour exists.
     */
    public double getAngle() {
        if (last) {
            return AngleUnit.normalizeRadians(angleOut);
        } else {
            return 14343;
        }
    }
}
