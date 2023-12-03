package org.firstinspires.ftc.teamcode.subsystems.Controllers;

import com.arcrobotics.ftclib.controller.PIDController;

public class IQIDController extends PIDController {

    public IQIDController(double kp, double ki, double kd) {
        super(kp, ki, kd);
    }

    public IQIDController(double kp, double ki, double kd, double sp, double pv) {
        super(kp, ki, kd, sp, pv);
    }

    @Override
    public double calculate(double pv) {
        double val = super.calculate(pv);
        return Math.signum(val)*Math.sqrt(Math.abs(val));
    }
}
