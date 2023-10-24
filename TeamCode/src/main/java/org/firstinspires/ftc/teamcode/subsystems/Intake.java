package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    MotorEx intakeMotor;
    double oscPower = 1;
    public Intake(HardwareMap hMap) {
        intakeMotor = new MotorEx(hMap, "intake");
    }
    public void intake(double power) {
        intakeMotor.set(power);
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
