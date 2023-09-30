package org.firstinspires.ftc.teamcode.drivers;

import com.arcrobotics.ftclib.drivebase.RobotDrive;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;

import java.util.ArrayList;

public class MultiHolonomic extends RobotDrive {
    private Motor[] motors;
    private ArrayList<Double> angles = new ArrayList<>();
    private boolean velocity=false;

    public MultiHolonomic(Motor[] motors, double[] angles) {
        this.motors=motors;
        for (double d : angles) {
            this.angles.add(d);
        }
    }

    public MultiHolonomic(Motor[] motors) {
        this.motors=motors;
        for (int i=0;i<motors.length;i++) {
            this.angles.add(Math.toRadians(360/motors.length*i));
        }
    }
    public MultiHolonomic(Motor[] motors, double offset) {
        this.motors=motors;
        for (int i=0;i<motors.length;i++) {
            this.angles.add(Math.toRadians((360/motors.length*i+offset)));
        }
    }

    public void setMaxSpeed(double value) {
        super.setMaxSpeed(value);
    }
    public void setRange(double min, double max) {
        super.setRange(min, max);
    }

    public void driveFieldCentric(double strafeSpeed, double forwardSpeed, double turn, double heading) {
        strafeSpeed = clipRange(strafeSpeed);
        forwardSpeed = clipRange(forwardSpeed);
        turn = clipRange(turn);

        Vector2d vector = new Vector2d(strafeSpeed, forwardSpeed);
        vector = vector.rotateBy(-heading);
        double theta = vector.angle();

        double[] speeds = new double[motors.length];

        for (int i=0;i<motors.length;i++) {
            speeds[i] = Math.cos(theta+angles.get(i));


        }
        normalize(speeds,vector.magnitude());
        double max_speed =0.0;
        for(int i = 0; i < motors.length; i++) {
            speeds[i] = maxOutput*(speeds[i] + turn);
            if(Math.abs(speeds[i]) > max_speed)
                max_speed = Math.abs(speeds[i]);
        }
        if(max_speed > 1.0) {
            double multiplier = 1.0 / max_speed;
            for (int i = 0; i < motors.length; i++) {
                motors[i].set(speeds[i] * multiplier);
            }
        }
        else {
            for (int i = 0; i < motors.length; i++) {
                motors[i].set(speeds[i]);
            }
        }
    }
    public void driveFieldCentric(Vector2d vector, double turn, double heading) {

        turn = clipRange(turn);


        vector = vector.rotateBy(-heading);
        double theta = vector.angle();

        double[] speeds = new double[motors.length];

        for (int i=0;i<motors.length;i++) {
            speeds[i] = Math.cos(theta+angles.get(i));


        }
        normalize(speeds,vector.magnitude());
        double max_speed =0.0;
        for(int i = 0; i < motors.length; i++) {
            speeds[i] = maxOutput*(speeds[i] + turn);
            if(Math.abs(speeds[i]) > max_speed)
                max_speed = Math.abs(speeds[i]);
        }
        if(max_speed > 1.0) {
            double multiplier = 1.0 / max_speed;
            for (int i = 0; i < motors.length; i++) {
                motors[i].set(speeds[i] * multiplier);
            }
        }
        else {
            for (int i = 0; i < motors.length; i++) {
                motors[i].set(speeds[i]);
            }
        }
    }
    public void driveRobotCentric(double strafeSpeed, double forwardSpeed, double turn) {
        driveFieldCentric(strafeSpeed, forwardSpeed, turn, 0.0);
    }
    public void setMode(boolean velocity) {
        this.velocity = velocity;
        if (velocity) {
            for (Motor m : motors) {
                m.setRunMode(Motor.RunMode.VelocityControl);
                m.setFeedforwardCoefficients(0.02, 0.0103);
                m.setVeloCoefficients(0.01, 0.00, 0.0);
            }
        }
        else
            for (Motor m : motors) {
                m.setRunMode(Motor.RunMode.RawPower);
            }
    }



    @Override
    public void stop() {
        for (Motor m : motors) {
            m.stopMotor();
        }
    }
    public double getMaxSpeed() {
        return maxOutput;
    }
}
