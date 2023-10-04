package org.firstinspires.ftc.teamcode.subsystems;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drivers.APDS9960;
import org.firstinspires.ftc.teamcode.drivers.AnalogEncoder;
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

}
