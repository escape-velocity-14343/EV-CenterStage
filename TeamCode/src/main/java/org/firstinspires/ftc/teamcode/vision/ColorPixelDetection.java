package org.firstinspires.ftc.teamcode.vision;
public class ColorPixelDetection {
    public static final int WHITE = 0;
    public static final int GREEN = 1;
    public static final int PURPLE = 2;
    public static final int YELLOW = 3;

    public int type;

    public double confidence;

    public ColorPixelDetection(int type) {
        if (type < 0 || type > 4) {
            throw new Error("That is not a valid type.");
        }
        this.type = type;
        this.confidence = 1;
    }

    public ColorPixelDetection(int type, double confidence) {
        if (type < 0 || type > 4) {
            throw new Error("That is not a valid type.");
        }
        this.type = type;
        this.confidence = confidence;
    }
}
