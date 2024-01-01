package org.firstinspires.ftc.teamcode.controllers;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pathutils.AsymmetricMotionProfile;

public class SquIDVAController {
    private SquIDController squid;
    private VAController va;

    private double SquIDTolerance;
    private double lastError = 0;
    private double setPoint = 0;
    private boolean hasRun = false;
    private ElapsedTime deltaTime = new ElapsedTime();

    /**
     * @param squidtol Tolerance for SquID controller to be used.
     */
    public SquIDVAController(double kP, double kI, double kD, double kV, double kA, double squidtol) {
        this.squid = new SquIDController(kP, kI, kD);
        this.va = new VAController(kV, kA);
        this.SquIDTolerance = squidtol;
    }

    public void setSquID(double kP, double kI, double kD) {
        this.squid.setPID(kP, kI, kD);
    }

    public void setVA(double kV, double kA) {
        this.va.setVA(kV, kA);
    }

    public void setMotionProfile(AsymmetricMotionProfile mp) {
        this.va.setMotionProfile(mp);
    }

    public double get(double sp, double pv) {
        double error = sp-pv;
        double out = 0;
        if(!hasRun) {

            lastError = error;
            hasRun = true;
            out = this.squid.calculate(error);
            if (error < SquIDTolerance) {
                out += this.va.calculate(error, 0);
            }
        } else {
            out = this.va.calculate(error, (lastError-error)/(deltaTime.seconds()));
            if (error < SquIDTolerance) {
                out += this.squid.calculate(error);
            }
        }
        deltaTime.reset();
        return out;
    }

    public double get(double pv) {
        return this.get(setPoint, pv);
    }

    public void setSetPoint(double sp) {
        this.setPoint = sp;
    }
}
