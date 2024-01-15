package org.firstinspires.ftc.teamcode.vision;

import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.util.Log;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
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

import java.util.concurrent.atomic.AtomicReference;

@Config
public class TeamPropProcessor implements VisionProcessor {
    //private final AtomicReference<Bitmap> lastFrame =
    //        new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));

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
    private Mat thres1 = new Mat();
    private Mat thres2 = new Mat();
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
    public static int blueRightOffset = -10;
    public static int blueHue = 110;
    public static int redHue = 10;
    public static int redHue2 = 170;
    Scalar white = new Scalar(255,255,255);
    public static Scalar blueMax = new Scalar(90, 0, 0);
    public static Scalar blueMin = new Scalar(150, 255, 255);
    public static Scalar redMin = new Scalar (170,0,0);
    public static Scalar redMax = new Scalar (20, 255, 255);
    public static boolean useHueThreshold = false;
    public static int satorvalue = 1;
    Mat ouput = new Mat();
    public static int greenoffset = 0;
    Bitmap bitmpaa;

    public TeamPropProcessor(boolean team) {
        this.team = team;
    }

    @Override
    public void init(int width, int height, CameraCalibration calib) {
        Log.println(Log.ASSERT,"michael", "wang");
        //lastFrame.set(Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565));
    }

    @Override
    /**
     *
     */
    public Object processFrame(Mat input, long captureTimeNanos) {
        Core.subtract(input,new Scalar(0,0,greenoffset),ouput);


        //if (team == TeamPropProcessor.BLUE) {

         Imgproc.cvtColor(ouput, HSV, Imgproc.COLOR_RGB2HSV);
         /*if (useHueThreshold) {
            if (!team) {
                Core.inRange(HSV, blueMin,blueMax,thres1);
            }
            else {
                Core.inRange(HSV,redMin, new Scalar(180,255,255),thres1);
                Core.inRange(HSV,new Scalar(0,0,0),redMax,thres2);
                Core.bitwise_or(thres1,thres2,thres1);
            }
            Core.bitwise_and(HSV,thres1,HSV);
         }*/
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
            int referenceColor = 0;
            if (team) {
                index = 2;
                referenceColor = redHue;
            } else {
                index = 2;
                referenceColor = blueHue;
            }
            index = satorvalue;

            //double leftavg = Core.mean(leftMat).val[index]+Core.mean(leftMat).val[1];
            //double midavg = Core.mean(midMat).val[index]+Core.mean(midMat).val[1];
            //double rightavg = Core.mean(rightMat).val[index]+Core.mean(rightMat).val[1];
            double leftavg = Core.mean(leftMat).val[index];//-Math.abs(referenceColor - Core.mean(leftMat).val[0]);
            double midavg = Core.mean(midMat).val[index];//-Math.abs(referenceColor - Core.mean(midMat).val[0]);
            double rightavg = Core.mean(rightMat).val[index];//-Math.abs(referenceColor - Core.mean(rightMat).val[0]);
            /*if (team) {
                leftavg -= Math.abs(redHue2 - Core.mean(leftMat).val[0]);
                midavg -= Math.abs(redHue2 - Core.mean(midMat).val[0]);
                rightavg -= Math.abs(redHue2 - Core.mean(rightMat).val[0]);
            }*/
            avgs[0] = leftavg;
            avgs[1] = midavg;
            avgs[2] = rightavg;
            Imgproc.rectangle(ouput,leftrect,white,2);
            Imgproc.rectangle(ouput,midrect,white,2);
            Imgproc.rectangle(ouput,rightrect,white,2);
            if (leftavg > midavg && leftavg > rightavg) {
                Imgproc.rectangle(ouput,leftrect,Core.mean(leftMat));
                placement = 0;
                return (Integer) 0;
            } else if (midavg > leftavg && midavg > rightavg) {
                Imgproc.rectangle(ouput,leftrect,Core.mean(midMat));
                placement = 1;
                return (Integer) 1;
            } else {
                Imgproc.rectangle(ouput,leftrect,Core.mean(rightMat));
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
        //lastFrame.set(bitmapFromMat(ouput));
        //bitmpaa = bitmapFromMat(ouput);

       return "fhoaiowiaejsdklf";
    }




    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        //canvas = new Canvas(bitmpaa);
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
        Bitmap b = Bitmap.createBitmap(mat.width(), mat.height(), Bitmap.Config.RGB_565);
        Utils.matToBitmap(mat, b);
        return b;
    }
    public void setTeam(boolean team) {
        this.team = team;
    }


    //@Override
    //public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
     //   continuation.dispatch(bitmapConsumer -> bitmapConsumer.accept(lastFrame.get()));
    //}
}
