package org.firstinspires.ftc.teamcode.vision;

import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.util.Log;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.CvException;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

@Config
public class TeamPropProcessor implements VisionProcessor {

    /**
     * Use TeamPropProcessor.RED or TeamPropProcessor.BLUE.
     * Values: RED = true, BLUE = false.
     */
    boolean team;

    /**
     * For team indication.
     */
    public static final boolean RED = true;

    /**
     * For team indication.
     */
    public static final boolean BLUE = false;

    public static final int LEFT = 0;
    public static final int MIDDLE = 1;
    public static final int RIGHT = 2;

    public static int blueval = 120;

    int camwidth;
    int camheight;

    private Mat YCrCb = new Mat();
    private Mat HSV = new Mat();
    private Rect leftrect;
    private Rect midrect;
    private Rect rightrect;

    private Integer placement = 1;

    private double[] avgs = new double[3];

    public static int leftCenterX = 180;
    public static int leftCenterY = 590;
    public static int leftX = 120;
    public static int leftY = 90;
    public static int leftRotation = 0;
    public static int midCenterX = 480;
    public static int midCenterY = 570;
    public static int midX = 120;
    public static int midY = 70;
    public static int midRotation = 0;
    public static int rightCenterX = 770;
    public static int rightCenterY = 560;
    public static int rightX = 100;
    public static int rightY = 100;
    public static int rightRotation = 0;

    public static int blueLeftOffset = -40;
    public static int blueMiddleOffset = -20;
    public static int blueRightOffset = 0;
    Scalar white = new Scalar(255,255,255);
    Mat ouput = new Mat();

    public TeamPropProcessor(boolean team) {
        this.team = team;
    }

    @Override
    public void init(int width, int height, CameraCalibration calib) {
        Log.println(Log.ASSERT,"michael", "wang");
    }

    @Override
    /**
     *
     */
    public Object processFrame(Mat input, long captureTimeNanos) {
        ouput = input.clone();

        //if (team == TeamPropProcessor.BLUE) {
        Imgproc.cvtColor(input, HSV, Imgproc.COLOR_RGB2HSV);
        //}
        //Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
        try {
            leftrect = new Rect(new Point(leftCenterX/2 + blueLeftOffset*(this.team==RED?0:1),leftCenterY/2),new Size (leftX/2,leftY/2));
            midrect = new Rect(new Point(midCenterX/2 + blueMiddleOffset*(this.team==RED?0:1),midCenterY/2),new Size (midX/2,midY/2));
            rightrect = new Rect(new Point(rightCenterX/2 + blueRightOffset*(this.team==RED?0:1),rightCenterY/2),new Size (rightX/2,rightY/2));



            Mat leftMat = new Mat(input.size(),input.type());
            leftMat = HSV.submat(leftrect);
            //Imgproc.circle(leftMat,new Point(leftCenterX,leftCenterY),leftX,white,40);
            Mat midMat = new Mat(input.size(),input.type());
            midMat = HSV.submat(midrect);
            //Imgproc.circle(midMat,new Point(midCenterX,midCenterY),midX,white,40);
            Mat rightMat = new Mat(input.size(),input.type());
            rightMat = HSV.submat(rightrect);
            //Imgproc.circle(rightMat,new Point(rightCenterX,rightCenterY),rightX,white,40);
            int index = 0;
            if (team) {
                index = 1;
            } else {
                index = 1;
            }

            //double leftavg = Core.mean(leftMat).val[index]+Core.mean(leftMat).val[1];
            //double midavg = Core.mean(midMat).val[index]+Core.mean(midMat).val[1];
            //double rightavg = Core.mean(rightMat).val[index]+Core.mean(rightMat).val[1];
            double leftavg = Core.mean(leftMat).val[index];
            double midavg = Core.mean(midMat).val[index];
            double rightavg = Core.mean(rightMat).val[index];

            avgs[0] = leftavg;
            avgs[1] = midavg;
            avgs[2] = rightavg;
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
           // if (leftavg > Math.max(midavg, rightavg)) {

           // }

            /*if (team) {
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
            } else {
                leftavg = Math.abs(leftavg-blueval);
                midavg = Math.abs(midavg-blueval);
                rightavg = Math.abs(rightavg-blueval);
                if (leftavg < midavg && leftavg < rightavg) {
                    placement = 0;
                    return (Integer) 0;
                } else if (midavg < leftavg && midavg < rightavg) {
                    placement = 1;
                    return (Integer) 1;
                } else {
                    placement = 2;
                    return (Integer) 2;
                }
            }*/

        } catch (Exception e) {
        }

       return "fhoaiowiaejsdklf";
    }




    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        Paint rectPaint = new Paint();
        rectPaint.setColor(Color.GREEN);
        rectPaint.setStyle(Paint.Style.STROKE);
        rectPaint.setStrokeWidth(scaleCanvasDensity * 4);

        canvas.drawRect(toGraphicsRect(leftrect, scaleBmpPxToCanvasPx), rectPaint);
        canvas.drawRect(toGraphicsRect(midrect, scaleBmpPxToCanvasPx), rectPaint);
        canvas.drawRect(toGraphicsRect(rightrect, scaleBmpPxToCanvasPx), rectPaint);

        rectPaint.setColor(Color.BLUE);

        switch (placement) {
            case 0:
                canvas.drawRect(toGraphicsRect(leftrect, scaleBmpPxToCanvasPx), rectPaint);
                break;
            case 1:
                canvas.drawRect(toGraphicsRect(midrect, scaleBmpPxToCanvasPx), rectPaint);
                break;
            case 2:
                canvas.drawRect(toGraphicsRect(rightrect, scaleBmpPxToCanvasPx), rectPaint);
                break;
        }

    }

    /**
     * 0: Left.
     * 1: Middle.
     * 2: Right.
     */
    public Integer getPosition() {
        return placement;
    }

    public double[] getVals() {
        return avgs;
    }

    private android.graphics.Rect toGraphicsRect(Rect rect, float scaleBmpPxToCanvasPx) {
        int left = Math.round(rect.x * scaleBmpPxToCanvasPx);
        int top = Math.round(rect.y * scaleBmpPxToCanvasPx);
        int right = Math.round((rect.x+rect.width)*scaleBmpPxToCanvasPx);
        int bottom = Math.round((rect.y+rect.height)*scaleBmpPxToCanvasPx);

        return new android.graphics.Rect(left, top, right, bottom);

    }
    public Bitmap bitmapFromMat(Mat mat) {
        Bitmap bmp = Bitmap.createBitmap(mat.width(), mat.height(), Bitmap.Config.ARGB_8888);
        Mat tmp = new Mat (mat.size(), CvType.CV_8U, new Scalar(4));
        try {
            //Imgproc.cvtColor(seedsImage, tmp, Imgproc.COLOR_RGB2BGRA);
            Imgproc.cvtColor(mat, tmp, Imgproc.COLOR_GRAY2RGBA, 4);
            bmp = Bitmap.createBitmap(tmp.cols(), tmp.rows(), Bitmap.Config.ARGB_8888);
            Utils.matToBitmap(tmp, bmp);
        }
        catch (CvException e){Log.d("Exception",e.getMessage());}
        return bmp;
    }
    public void setTeam(boolean team) {
        this.team = team;
    }


}
