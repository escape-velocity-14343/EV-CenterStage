package org.firstinspires.ftc.teamcode.subsytems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.cachinghardwaredevice.CachingServo;
import org.firstinspires.ftc.teamcode.wrappers.CachedServo;

@Config
public class Bucket {
    // TODO: entire class needs implementation and testing on hardware
    private CachingServo leftLatch;
    private Servo rightLatch;
    private final double latchPos = 0;
    private final double unlatchPos = 1;
    private final double doublePixelPos = 0.5;


    public void init(HardwareMap hmap) {
        leftLatch = hmap.get(CachingServo.class, "leftLatch");
        rightLatch = hmap.get(CachingServo.class, "rightLatch");
    }

    public void latch() {
        leftLatch.setPosition(latchPos);
        rightLatch.setPosition(latchPos);
    }

    public void unlatch() {
        leftLatch.setPosition(unlatchPos);
        rightLatch.setPosition(unlatchPos);
    }

    public void dropFromStack() {
        leftLatch.setPosition(doublePixelPos);
        rightLatch.setPosition(doublePixelPos);
    }

}
