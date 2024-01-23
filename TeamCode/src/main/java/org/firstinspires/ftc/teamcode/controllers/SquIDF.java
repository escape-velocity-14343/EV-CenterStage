package org.firstinspires.ftc.teamcode.controllers;

import android.util.Log;

public class SquIDF extends SquIDController {

    private AdaptableStaticGain kStatic;

    public SquIDF(double kP, double kI, double kD, double kF) {
        super(kP, kI, kD);
        kStatic = new AdaptableStaticGain(kF, 0, 0);
    }

    public SquIDF(double kP, double kI, double kD, double kF, double increment, double decrement) {
        this(kP, kI, kD, 0);
        kStatic = new AdaptableStaticGain(kF, increment, decrement);
    }

    public void setFeedforwardBounds(double min, double max) {
        kStatic.setBounds(min, max);
    }

    public void setFeedforwardTolerance(double tolerance) {
        kStatic.setIncrementTolerance(tolerance);
    }

    @Override
    public double calculate(double pv) {
        double out = super.calculate(pv);
        kStatic.update(pv, getSetPoint());
        Log.println(Log.INFO, "SquIDF", "Kstatic value: " + kStatic.calculate());
        Log.println(Log.INFO, "SquIDF", "Error: " + (getSetPoint()-pv));
        return out + Math.signum(out) * kStatic.calculate();
    }

    public double calculate(double pv, double sp, double tolerance) {
        double error = sp - pv;
        double out = super.calculate(pv, sp);
        kStatic.update(pv, getSetPoint(), tolerance);
        double ff = kStatic.calculate();
        Log.println(Log.INFO, "SquIDF", "Kstatic value: " + ff);
        Log.println(Log.INFO, "SquIDF", "Error: " + (getSetPoint()-pv));
        if (Math.abs(error) < tolerance) {
            return 0;
        }
        return out + Math.signum(out) * ff;
    }

    public void forceDecrement() {
        kStatic.forceDecrement();
    }
}
