package org.firstinspires.ftc.teamcode.vision;

import android.graphics.Canvas;

import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

class PipelineKiwi implements VisionProcessor {
    boolean viewportPaused;
    Mat dst = new Mat();
    Mat hsv = new Mat();
    Mat tresh = new Mat();
    Mat hierarchy = new Mat();
    Scalar color = new Scalar(0,255,0);
    int pos = 0;
    double maxArea = 0;
    double maxWidth = 0;
    double maxHeight = 0;
    int maxIndex = 0;
    MatOfPoint maxContour;
    DetectType type = DetectType.POLE;
    Scalar minYellow = new Scalar(15,100,128);
    Scalar maxYellow = new Scalar(50,255,255);
    Scalar minBlue = new Scalar(90,60,50);
    Scalar maxBlue = new Scalar(130,255,255);
    Scalar minBlack = new Scalar(1,1,0);
    Scalar maxBlack = new Scalar(255,255,255);
    //Scalar minBlue = new Scalar(104,100,60);
    //Scalar maxBlue = new Scalar(130,255,255);
    //    Scalar minBlue = new Scalar(110,90,45);
    //    Scalar maxBlue = new Scalar(130,255,255);
    Scalar minLowRed = new Scalar(0,90,45);
    Scalar maxLowRed = new Scalar(15,255,255);
    Scalar minHighRed = new Scalar(170,90,45);
    Scalar maxHighRed = new Scalar(180,255,255);
    Mat lowRed = new Mat();
    Mat highRed = new Mat();
    int maxX = 0;
    int maxY = 0;
    int resMultiplier = 1;
    MatOfPoint2f contourMat;
    MatOfPoint2f thing;
    MatOfPoint2f approx;
    List<MatOfPoint> contours;
    NormalizedRGBA normalizedRGBA;
    Scalar drawColor = new Scalar(0,0,0);
    int width = -404;
    int height = -404;

    /*
     * NOTE: if you wish to use additional Mat objects in your processing pipeline, it is
     * highly recommended to declare them here as instance variables and re-use them for
     * each invocation of processFrame(), rather than declaring them as new local variables
     * each time through processFrame(). This removes the danger of causing a memory leak
     * by forgetting to call mat.release(), and it also reduces memory pressure by not
     * constantly allocating and freeing large chunks of memory.
     */

    @Override
    public Mat processFrame(Mat input, long captureTimeNanos) {
        if (input.width()>400)
            resMultiplier = 1;
        else
            resMultiplier = 2;
        if (this.type== DetectType.OFF)
            return input;
        /*
         * IMPORTANT NOTE: the input Mat that is passed in as a parameter to this method
         * will only dereference to the same image for the duration of this particular
         * invocation of this method. That is, if for some reason you'd like to save a copy
         * of this particular frame for later use, you will need to either clone it or copy
         * it to another Mat.
         */

        contours = new ArrayList<>();
        Imgproc.GaussianBlur(input, dst, new Size(5,5), 0);
        Imgproc.cvtColor(dst, hsv, Imgproc.COLOR_RGB2HSV);
        if (type == DetectType.POLE)
            Core.inRange(hsv, minYellow, maxYellow, tresh);
        else if (type == DetectType.BLUE) {
            Core.inRange(hsv, minBlue, maxBlue, tresh);
        }
        else if (type == DetectType.BLACK) {
            Core.inRange(hsv, minBlack, maxBlack, tresh);
        }
        else {
            Core.inRange(hsv, minLowRed, maxLowRed, lowRed);
            Core.inRange(hsv, minHighRed, maxHighRed, highRed);
            Core.bitwise_or(lowRed,highRed,tresh);
        }

        Imgproc.findContours(tresh, contours, hierarchy, Imgproc.RETR_TREE
                , Imgproc.CHAIN_APPROX_NONE);
        maxArea=0;
        maxContour=null;
        contourMat = new MatOfPoint2f();
        RotatedRect maxRect = new RotatedRect();
        maxWidth = 0;
        maxHeight = 0;
        double width=0;
        double height=0;

        for(int i = 0; i < contours.size(); i++) {
            double area = Imgproc.contourArea(contours.get(i));
            try {
                contourMat = new MatOfPoint2f(contours.get(i).toArray());
                maxRect = Imgproc.minAreaRect(contourMat);
                width = maxRect.boundingRect().width;
                height = maxRect.boundingRect().height;
                //Log.println(Log.INFO,"me","good job :D "+i+" "+contours.size());
            }
            catch(Exception e) {
                //Log.println(Log.ERROR,"im still here >:D","aughhhh "+i+" "+contours.size());
            }
            boolean biggest = false;
            if (type == DetectType.POLE) {
                biggest = (area>800) && (area > maxArea && width < height / 2);
            }
            else {
                biggest = (area>800) && area>maxArea;
            }
            if (biggest) {
                maxArea=area;
                maxWidth = width;
                maxContour=contours.get(i);
            }
            if(area>800){
                Imgproc.drawContours(input, contours, -1, color, 2
                        , Imgproc.LINE_8, hierarchy, 2, new Point());
                MatOfPoint point2 = contours.get(i);
                double[] position = point2.get(0,0);
                int x = (int) position[0];
                pos = x;
            }
        }
        if (maxContour!=null) {
            thing = new MatOfPoint2f(maxContour.toArray());
            RotatedRect rect = Imgproc.minAreaRect(thing);
            approx = new MatOfPoint2f();
            Imgproc.approxPolyDP(thing, approx,0.15*Imgproc.arcLength(thing,true),true);
            if (!approx.empty()) {
                for (Point p : approx.toArray()) {
                    Imgproc.drawMarker(input,p,new Scalar(2,20,200));
                }
            }
            Imgproc.drawMarker(input,rect.center,new Scalar(200,20,2));
            maxX = (int) rect.center.x*resMultiplier;
            maxY = (int) rect.center.y*resMultiplier;
            width = maxContour.width();
            height = maxContour.height();
        }
        else {
            maxX = -404;
            maxY = -404;
        }
        maxContour=null;
        contourMat=null;
        return input;
    }
    public int getPos() {
        return pos;
    }
    public MatOfPoint getMaxContour() {
        return maxContour;
    }
    public int getX() {
       return maxX;
    }
    public int getY() {
        return maxY;
    }
    public int getWidth() {
        return width;
    }
    public int getHeight() {
        return height;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }



    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }

    public enum DetectType{
        POLE,
        BLUE,
        RED,
        BLACK,
        OFF
    }
    public void setColor(DetectType type) {
        this.type = type;
    }
    public void setMinColor(Scalar color) {
        if (type== DetectType.POLE)
            this.minYellow=color;
        else if (type== DetectType.BLUE)
            this.minBlue=color;
        else if (type== DetectType.RED)
            this.minHighRed=color;
    }
    public void setMaxColor(Scalar color) {
        if (type== DetectType.POLE)
            this.maxYellow=color;
        else if (type== DetectType.BLUE)
            this.maxBlue=color;
        else if (type== DetectType.RED)
            this.maxHighRed=color;
    }
    public void drawColor(Scalar color) {
        this.drawColor= color;
    }


}