package org.firstinspires.ftc.teamcode.controllers;

import com.arcrobotics.ftclib.controller.PIDController;

// goofy asf sqrt pid control
// #SquIDgame
public class SquIDController extends PIDController {
    public SquIDController(double kP, double kI, double kD) {
        super(kP, kI, kD);
    }

    @Override
    public double calculate(double pv) {
        double out = super.calculate(pv);
        return Math.signum(out)*Math.sqrt(Math.abs(out));
    }


}
