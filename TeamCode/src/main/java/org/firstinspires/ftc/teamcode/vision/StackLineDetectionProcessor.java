package org.firstinspires.ftc.teamcode.vision;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.util.Log;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Range;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

@Config
public class StackLineDetectionProcessor implements VisionProcessor {

    public static int row = 100;

    private int width;
    private int height;

    private int startCol = 0;
    private int endCol = 0;


    Mat HSV;

    public void init(int width, int height, CameraCalibration calib) {
        Log.println(Log.INFO, "weeee", "states lighting better not screw this up");
        this.width = width;
        this.height = height;
    }

    public Object processFrame(Mat frame, long captureTimeNanos) {
        // convert to HSV
        Imgproc.cvtColor(frame, HSV, Imgproc.COLOR_RGB2HSV);

        // get the row
        Mat HSVsub = HSV.submat(new Range(row, row+1), Range.all());

        // subtract the average of the single row
        double avg = Core.mean(HSVsub).val[2];
        Core.subtract(HSVsub, new Scalar(0, 0, avg), HSVsub);

        // run kadane's algorithm
        double sum = 0;
        int tail = 0; // inclusive
        int head = 0; // exclusive
        double maxSum = 0;
        int maxTail = 0; // inclusive
        int maxHead = 0; // exclusive

        for (int col = 0; col < width; col++) {
            double val = HSVsub.get(0, col)[2];
            sum += val;
            head++;
            if (sum > maxSum) {
                maxSum = sum;
                maxTail = tail;
                maxHead = head;
            }
            if (sum < 0) {
                sum = 0;
                tail = head;
            }

        }

        startCol = maxTail;
        endCol = maxHead;

        return null;


    }

    public double getMiddle() {
        return (startCol + endCol) / 2.0;
    }

    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        Paint rectPaint = new Paint();
        rectPaint.setColor(Color.GREEN);
        rectPaint.setStyle(Paint.Style.STROKE);
        rectPaint.setStrokeWidth(scaleCanvasDensity * 4);

        canvas.drawRect(toGraphicsRect(new Rect(startCol, 0, endCol-startCol, height), scaleBmpPxToCanvasPx), rectPaint);
    }

    private android.graphics.Rect toGraphicsRect(Rect rect, float scaleBmpPxToCanvasPx) {
        int left = Math.round(rect.x * scaleBmpPxToCanvasPx);
        int top = Math.round(rect.y * scaleBmpPxToCanvasPx);
        int right = Math.round((rect.x+rect.width)*scaleBmpPxToCanvasPx);
        int bottom = Math.round((rect.y+rect.height)*scaleBmpPxToCanvasPx);

        return new android.graphics.Rect(left, top, right, bottom);

    }
}
