package org.firstinspires.ftc.teamcode.subsystems;



import static org.firstinspires.ftc.teamcode.subsystems.SwerveController.headingwheelratio;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Rotation;
import org.firstinspires.ftc.teamcode.drivers.AnalogEncoder;

import java.util.Arrays;

public class SwerveModule {

    Motor top,bottom;
    AnalogEncoder rot;
    Telemetry telemetry;
    PIDController pid = new PIDController(0.01, 0.0, 0.02);
    boolean right=false;
    String direction = "";
    double error = 0.0;

    int rotationValueWindow = 7;
    double[] rotationValues = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    int rotationValueIndex = 0;
    double lastRotation;

    public SwerveModule(Motor bottom, Motor top, AnalogEncoder rot, Telemetry telemetry) {
        this.top = top;
        this.bottom = bottom;
        this.rot = rot;
        this.telemetry = telemetry;
    }
    /**
    *@param false = left, true = right
    */
    public void setSide(boolean right) {
        this.right = right;
        direction = (right? "right" : "left");
    }

    public void setPid(double p, double i, double d) {
        this.pid.setPID(p, i, d);
    }
    /**
    * Moves the pod in a specific direction, with a specific speed
    * @param vector the vector to move the pod in, relative to the robot
    */
    public void podPid(Vector2d vector) {
        podPid(vector.magnitude(), vector.angle());
    }
    /**
    * Moves the pod in a specific direction, with a specific speed
    * @param x the x value to move the pod in, relative to the robot
    * @param y the y value to move the pod in, relative to the robot
    */
    public void podPidXY(double x, double y) {
        podPid(Math.hypot(x,y), Math.toDegrees(Math.atan2(y,x)));
    }
    public void AddRotation(double rotation) {
        rotationValues[rotationValueIndex] = rotation;
        rotationValueIndex = (rotationValueIndex + 1) % rotationValueWindow;
    }
    public double getMedianRotation() {
        double array[] = rotationValues.clone();
       Arrays.sort(array);
       return array[rotationValueWindow/2];
    }
    /**
    * Moves the pod in a specific direction, with a specific speed
    * @param wheel the speed to drive the pod
    * @param heading the direction to move the pod in, relative to the robot
    */
    public void podPid(double wheel, double heading) {
        double rotation = rot.getDegrees();
        telemetry.addData("lastRotation"+direction, lastRotation);
        lastRotation = rotation;
        telemetry.addData("RawRotation"+direction, rotation);
        AddRotation(rotation);
        //rotation = getMedianRotation();
        telemetry.addData("SmoothedRotation"+direction, getMedianRotation());
        double moveTo = AngleUnit.normalizeDegrees(heading-rotation);
        if (!compare(moveTo,0.0,90.0)) {
            moveTo = AngleUnit.normalizeDegrees(moveTo-180.0);
            wheel*=-1;
        }
        //telemetry.addData("moveTo"+direction, moveTo);
        //telemetry.addData("rotation"+direction, rotation);
        //telemetry.addData("heading"+direction, heading);
        pid.setSetPoint(moveTo);
        error = pid.getPositionError();
        //telemetry.addData("error"+direction, error);
        double pidcalc = pid.calculate(0.0);
        //pidcalc = Math.signum(pidcalc) * Math.pow(pidcalc,2.0);
        //telemetry.addData("pidcalc"+direction, pidcalc);
        podMove(Math.cos(Math.toRadians(error))*wheel, pidcalc);
    }
    public void podMove(double wheel, double heading) {
        heading*=-1;

        //telemetry.addData("wheel"+direction, wheel);
        //telemetry.addData("headingpower"+direction, heading);
        wheel /= headingwheelratio;
        double topP = wheel-heading;
        double bottomP = wheel+heading;


        double [] powers = {topP, bottomP};
       // powers = normalize(powers, 1.0);
        //telemetry.addData("TopP"+direction, topP);
        //telemetry.addData("bottomP"+direction, bottomP);
        top.set(powers[0]);
        bottom.set(-powers[1]);
    }

    public double[] normalize(double[] values, double magnitude) {
        double maxMagnitude = Math.abs(values[0]);
        for (int i = 1; i < values.length; i++) {
            double temp = Math.abs(values[i]);
            if (maxMagnitude < temp) {
                maxMagnitude = temp;
            }
        }
        if (maxMagnitude>magnitude) {
            for (int i = 0; i < values.length; i++) {
                values[i] = (values[i] / maxMagnitude) * magnitude;
            }
        }
        return values;

    }
    public boolean close(double moveTo) {
        return compare(moveTo, rot.getDegrees(),10)||compare(moveTo, AngleUnit.normalizeDegrees(rot.getDegrees()+180), 10);
    }
    public double getError() {
        return error;
    }
    public static boolean compare(double a, double b, double tolerance) {
        return (Math.abs(a-b)<tolerance);
    }

}

