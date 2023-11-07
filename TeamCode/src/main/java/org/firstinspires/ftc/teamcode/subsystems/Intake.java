package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class Intake {
    Motor intakeMotor;
    DcMotorEx currentSensor;
    double oscPower = 1;
    public Intake(HardwareMap hMap) {
        intakeMotor = new Motor(hMap, "intake");
        currentSensor = hMap.get(DcMotorEx.class,"intake");
    }
    public void intake(double power) {
        if (getCurrentDraw()>Robot.intakeCurrentDraw) {
            intakeMotor.set(-0.8);
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


}
