package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class Intake {
    Motor intakeMotor, encoderMotor;

    Servo flipDown;
    DcMotorEx currentSensor;
    double oscPower = 1;
    double target;
    PIDController intakePID = new PIDController(0,0,0);
    public Intake(HardwareMap hMap) {
        intakeMotor = new Motor(hMap, "intake");
        encoderMotor = new Motor(hMap,"slide1");
        flipDown = hMap.servo.get("flip");

        currentSensor = hMap.get(DcMotorEx.class,"intake");
    }
    public void intake(double power) {
        if (getCurrentDraw()>Robot.intakeCurrentDraw) {
            intakeMotor.set(-1);
        }
        else
            intakeMotor.set(Range.clip(power, -1, 1));

    }
    public void armDown() {
        flipDown.setPosition(Robot.intakeFlipDown);
    }
    public void armUp() {
        flipDown.setPosition(Robot.intakeFlipUp);
    }
    public void armMove(double pos) {
        flipDown.setPosition(pos);
    }
    public double getCurrentDraw() {
        return currentSensor.getCurrent(CurrentUnit.AMPS);
    }

    public void intakeOscillating() {
        intakeMotor.set(oscPower);
        if (oscPower<1) {
            oscPower+=0.05;
        }
        else
            oscPower=0.5;
    }

    public int getPosition() {
        return -encoderMotor.getCurrentPosition();
    }
    public void stop() {
        intakeMotor.stopMotor();
    }
    public void pidIntake(int target) {
        this.target = target;
        intakePID.setPID(Robot.kIntakeP, Robot.kIntakeI, Robot.kIntakeD);
        intakePID.setSetPoint(target);
        intake(Range.clip(-intakePID.calculate(-encoderMotor.getCurrentPosition()), -0.4, 0.4));
    }

    public void brake() {
        intakeMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
    }

    public void endBrake() {
        intakeMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
    }
    public void reset() {
        encoderMotor.resetEncoder();
    }


}
