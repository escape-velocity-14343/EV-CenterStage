package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class Intake {
    Motor intakeMotor, motor0;
    DcMotorEx currentSensor;
    double oscPower = 1;
    double target;
    PIDController intakePID = new PIDController(0,0,0);
    public Intake(HardwareMap hMap) {
        intakeMotor = new Motor(hMap, "intake");
        //motor0 = new Motor(hMap,"slide1");
        //motor0.resetEncoder();
        currentSensor = hMap.get(DcMotorEx.class,"intake");
    }
    public void intake(double power) {
        if (getCurrentDraw()>Robot.intakeCurrentDraw) {
            intakeMotor.set(-1);
        }
        else
            intakeMotor.set(power);

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
    public void stop() {
        intakeMotor.stopMotor();
    }
    public void pidIntake(int target) {
        this.target = target;
        intakePID.setPID(Robot.kIntakeP, Robot.kIntakeI, Robot.kIntakeD);
        intakePID.setSetPoint(target);
        intake(intakePID.calculate(-motor0.getCurrentPosition()));
    }


}
