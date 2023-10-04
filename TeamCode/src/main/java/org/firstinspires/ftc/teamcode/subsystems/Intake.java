package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    MotorEx intakeMotor;
    public Intake(HardwareMap hMap) {
        intakeMotor = new MotorEx(hMap, "intake");
    }
    public void intake(double power) {
        intakeMotor.set(power);
    }

}
