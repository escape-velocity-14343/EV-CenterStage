package org.firstinspires.ftc.teamcode.vision;

import android.graphics.Bitmap;
import android.graphics.Canvas;

import com.acmerobotics.dashboard.config.Config;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.android.Utils;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

@Config
public class UndistortProcessor implements VisionProcessor {
    public static double fx = 356.182925829;
    public static double fy = 356.182925829;
    public static double cx = 319.833258237;
    public static double cy = 235.480453978;
    public static double k1 = -0.0311449206435;
    public static double k2 = 0.00974000885112;
    public static double k3 = -0.0013747840902;
    public static boolean toRun = true;
    Mat cameraMatrix;
    Mat deez;
    Bitmap b;
    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        constructMatrix();
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        if (toRun) {
            Calib3d.fisheye_undistortImage(frame, frame, cameraMatrix, deez, cameraMatrix);
        }
        b = Bitmap.createBitmap(frame.width(), frame.height(), Bitmap.Config.RGB_565);
        Utils.matToBitmap(frame, b);
        return this;

    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        canvas = new Canvas(b);
    }

    void constructMatrix()
    {
        //     Construct the camera matrix.
        //
        //      --         --
        //     | fx   0   cx |
        //     | 0    fy  cy |
        //     | 0    0   1  |
        //      --         --
        //

        cameraMatrix = new Mat(3,3, CvType.CV_32FC1);

        cameraMatrix.put(0,0, new float[]{(float) fx, 0, (float) cx});
        //cameraMatrix.put(0,1,0);
        //cameraMatrix.put(0,2, cx);

        cameraMatrix.put(1,0,new float[]{0, (float) fy, (float) cy});
        //cameraMatrix.put(1,1,fy);
        //cameraMatrix.put(1,2,cy);

        cameraMatrix.put(2, 0, new float[]{0, 0, 1});
        //cameraMatrix.put(2,1,0);
        //cameraMatrix.put(2,2,1);
        deez = new Mat(1, 4, CvType.CV_32FC1);
        deez.put(0, 0, new float[]{(float) k1, (float) k2, (float) k3, 0});
    }
}
