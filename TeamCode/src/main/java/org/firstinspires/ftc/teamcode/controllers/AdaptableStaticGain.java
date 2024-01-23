package org.firstinspires.ftc.teamcode.controllers;

import android.util.Log;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * A self-adapting static gain. Works as follows:
 * If the system is not moving and also not done, the static gain will be incremented.
 * If the system is overshooting (the sign of the error changes), the static gain will be decremented.
 * This ensures automatic and robust tuning, regardless of field condition.
 */
public class AdaptableStaticGain {
    /**
     * The static gain.
     */
    private double kStatic;

    /**
     * The amount the gain will be incremented by, on a per-second basis, when there is no change in error.
     */
    private double increment;
    /**
     * The amount the gain will be decremented by when the controller overshoots, per instance. By default this is the same as increment.
     */
    private double decrement;

    /**
     * The maximum possible change in error per second where the system is still not moving.
     */
    private double incrementTolerance = 0.01;

    private double maxGain = Double.POSITIVE_INFINITY;
    private double minGain = Double.NEGATIVE_INFINITY;

    private double lastError = Double.NaN;
    private double output = kStatic;
    private ElapsedTime timer;
    private long lastNanos;

    public AdaptableStaticGain(double kStatic, double increment) {
        this.kStatic = kStatic;
        this.increment = increment;
        this.decrement = increment;
        this.timer = new ElapsedTime();
        this.lastNanos = timer.nanoseconds();

    }

    public AdaptableStaticGain(double kStatic, double increment, double decrement) {
        this(kStatic, increment);
        this.decrement = decrement;
    }

    /**
     * Cap the possible values for the gain. Use {@code Double.POSITIVE_INFINITY} or {@code Double.NEGATIVE_INFINITY} to uncap bounds.
     */
    public void setBounds(double min, double max) {
        this.minGain = min;
        this.maxGain = max;
    }

    /**
     * @param tolerance The maximum possible change in error where the system is still not moving.
     */
    public void setIncrementTolerance(double tolerance) {
        this.incrementTolerance = tolerance;
    }

    public void update(double error) {
         // if we are within variance of target, skip update step

        long nanos = timer.nanoseconds();
        long deltaNanos = nanos - lastNanos;
        double deltaSeconds = ((double)(deltaNanos))/1e9;
        if (Math.abs(error) < incrementTolerance * deltaSeconds) {
            output = 0;
            lastNanos = nanos;
            //lastError = error;
            return;
        }
        if (Math.abs(error - lastError) < incrementTolerance * deltaSeconds) {
            Log.println(Log.INFO, "SquIDF", "Incrementing! Tolerance: " + incrementTolerance*deltaSeconds);
            kStatic += increment * deltaSeconds;
            } else if (Math.signum(error) != Math.signum(lastError)) {
                Log.println(Log.INFO, "SquIDF", "Decrementing!");
                kStatic -= decrement;
        }
        Log.println(Log.INFO, "SquIDF", "Error signum: " + Math.signum(error) + ", lastError signum: " + Math.signum(lastError));
        kStatic = Range.clip(kStatic, minGain, maxGain);
        if (Math.abs(error - lastError) < incrementTolerance * deltaSeconds) {
            output = kStatic;
        } else {
            output = 0;
        }
        lastNanos = nanos;
        lastError = error;

    }

    public void forceDecrement() {
        Log.println(Log.INFO, "SquIDF", "Decrementing!");
        kStatic -= decrement;
    }

    public void update(double pv, double sp, double tolerance) {
        if (Math.abs(sp-pv) > tolerance) {
            this.update(pv, sp);
        } else {
            long nanos = timer.nanoseconds();
            long deltaNanos = nanos - lastNanos;
            double deltaSeconds = ((double)(deltaNanos))/1e9;
            output = 0;
            lastNanos = nanos;
            //lastError = sp-pv;
            return;
        }
    }

    public void update(double pv, double sp) {
        this.update(sp-pv);
    }

    public double calculate() {
        return output;
    }
}
