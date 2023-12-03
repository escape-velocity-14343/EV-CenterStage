package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.Controllers.IQIDController;

public class Slides {
    public DcMotor motor0;
    public Motor motor2, motor1;
    Servo tilt;
    PIDController slidePID = new PIDController(0,0,0);
    IQIDController slideIQID = new IQIDController(0, 0, 0);
    int target;
    public Slides(HardwareMap hMap) {
        //m0 = hMap.dcMotor.get("slide0");
        motor0 = hMap.dcMotor.get("slide0"); // middle motor
        motor2 = new Motor(hMap, "xEnc"); // bottom motor
        motor1 = new Motor(hMap,"slide1"); // top motor

        motor0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor2.setRunMode(Motor.RunMode.RawPower);
        motor1.setRunMode(Motor.RunMode.RawPower);
        //motor2.setInverted(true);
        tilt = hMap.get(Servo.class,"slidetilt");
        //motor0.setInverted(true);


    }
    public void moveSlides(double power) {
        motor0.setPower(power);
        motor1.set(-power);
        motor2.set(-power);
    }
    public void pidSlides(int target) {
        // add corrective factor for change in cpr
        this.target = target;
        slidePID.setPID(Robot.kSlidesP, Robot.kSlidesI, Robot.kSlidesD);
        slideIQID.setPID(Robot.kSlidesP, Robot.kSlidesI, Robot.kSlidesD);
        slidePID.setSetPoint(target);
        slideIQID.setSetPoint(target);
        moveSlides(slidePID.calculate(getPosition())+Robot.kSlidesStatic);
    }

    public boolean pidWithTol(int target, int tolerance) {
        if (Math.abs(getPosition()-(target))<=tolerance) {
            return true;
        } else {
            pidSlides(target);
            return false;
        }
    }

    public boolean pidWithTol(int target, int tolerance, boolean turnOffPID) {
        if (!turnOffPID) {
            pidSlides(target);
        }
        if (Math.abs(getPosition()-(target))<=tolerance) {
            return true;
        } else {
            if (turnOffPID) {
                pidSlides(target);
            }
            return false;
        }
    }
    public int getPosition() {
        return -motor0.getCurrentPosition();
    }

    public void tilt(double angle) {
        tilt.setPosition(angle);
    }
    public void reset() {
        motor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //motor1.resetEncoder();
    }
}
