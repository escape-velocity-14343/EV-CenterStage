package org.firstinspires.ftc.teamcode.subsystems;


import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drivers.APDS9960;
import org.firstinspires.ftc.teamcode.drivers.AnalogEncoder;
import org.firstinspires.ftc.teamcode.vision.ColorPixelDetection;

@Config
public class Bucket {

    public static double intakePos = 0;
    public static double outtakePos = 1;
    public static double latchClosed = 0;
    public static double latchIntake = 0.5;
    public static double latchOpen = 1;
    public static double proxTresh = 5;

    Servo bucketServo;
    Servo latchServo;
    AnalogEncoder position;
    APDS9960 topSensor;
    APDS9960 bottomSensor;

    // bgr colorspace
    private static final double[] green = new double[]{0.33, 0.90, 0.18};
    private static final double[] yellow = new double[]{1.5, 2.1, 3.2};

    private static final double[] purple = new double[]{0.25, 0.09, 0.10};

    private static final double[] white = new double[]{1.35, 1.52, 1.23};


    public Bucket(HardwareMap hmap) {
        bucketServo = hmap.get(Servo.class, "bucket");
        latchServo = hmap.get(Servo.class,"latch");
        position = new AnalogEncoder(hmap, "bucketPos");
        topSensor = APDS9960.fromHMap(hmap, "bucketTop");
        bottomSensor = APDS9960.fromHMap(hmap, "bucketBottom");
    }

    public boolean intake() {
        bucketServo.setPosition(intakePos);
        latchServo.setPosition(latchIntake);
        return true;
    }


    public boolean outtake() {
        return true;
    }
    public int pixelsIn() {
        int pixels = 0;
        if (topSensor.getProximity()<proxTresh) {
            pixels = 2;
        }
        else if (bottomSensor.getProximity()<proxTresh) {
            pixels = 1;
        }
        return pixels;
    }

    public ColorPixelDetection[] getPixelColors() {
        int num = pixelsIn();
        if (num == 0) {
            return new ColorPixelDetection[0];
        } else if (num == 1) {
            return new ColorPixelDetection[]{getPixelColor(bottomSensor.getRed(), bottomSensor.getGreen(), bottomSensor.getBlue())};
        } else {
            return new ColorPixelDetection[]{getPixelColor(bottomSensor.getRed(), bottomSensor.getGreen(), bottomSensor.getBlue()), getPixelColor(topSensor.getRed(), topSensor.getGreen(), topSensor.getBlue())};
        }
    }

    private ColorPixelDetection getPixelColor(double r, double g, double b) {
        double[] dists = new double[4];
        double[] bgr = new double[]{b, g, r};
        Log.println(Log.INFO, "pixel", "" + r + " " + g + " " + b);
        dists[0] = getColorDist(bgr, white);
        dists[1] = getColorDist(bgr, green);
        dists[2] = getColorDist(bgr, purple);
        dists[3] = getColorDist(bgr, yellow);
        double min = 10000;
        int minind = -1;
        for (int i = 0; i < 4; i++) {
            if (dists[i] < min) {
                min = dists[i];
                minind = i;
            }
        }
        return new ColorPixelDetection(minind);
    }

    private double getColorDist(double[] color1, double[] color2) {
        assert color1.length == color2.length;
        double dist = 0;
        for (int i = 0; i < color1.length; i++) {
            dist += Math.pow(color1[i]-color2[i], 2);
        }
        return dist;
    }

}
