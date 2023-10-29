package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Slides {
    Motor motor0;
    Motor motor1;
    Servo tilt;
    PIDController slidePID = new PIDController(0,0,0);
    int target;
    public Slides(HardwareMap hMap) {
        motor0 = new Motor(hMap,"slide0");
        motor1 = new Motor(hMap,"slide1");
        tilt = hMap.get(Servo.class,"slidetilt");
        motor0.setInverted(true);
        motor0.resetEncoder();
        motor1.resetEncoder();

    }
    public void moveSlides(double power) {
        motor0.set(power);
        motor1.set(power);
    }
    public void pidSlides(int target) {
        this.target = target;
        slidePID.setPID(Robot.kSlidesP, Robot.kSlidesI, Robot.kSlidesD);
        slidePID.setSetPoint(target);
        moveSlides(slidePID.calculate(-motor0.getCurrentPosition())+Robot.kSlidesStatic);
    }
    public int getPosition() {
        return -motor0.getCurrentPosition();
    }

    public void tilt(double angle) {
        tilt.setPosition(angle);
    }
}
