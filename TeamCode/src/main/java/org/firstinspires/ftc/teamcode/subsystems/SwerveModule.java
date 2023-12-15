package org.firstinspires.ftc.teamcode.subsystems;




import static org.firstinspires.ftc.teamcode.subsystems.SwerveController.headingwheelratio;
import static org.firstinspires.ftc.teamcode.subsystems.SwerveController.kBottom;
import static org.firstinspires.ftc.teamcode.subsystems.SwerveController.kTop;
import static org.firstinspires.ftc.teamcode.subsystems.SwerveController.optimize;
import static org.firstinspires.ftc.teamcode.subsystems.SwerveController.rotationConstant;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.teamcode.drivers.AnalogEncoder;
import org.firstinspires.ftc.teamcode.drivers.ToggleTelemetry;

import java.util.Arrays;

public class SwerveModule {

    Motor top,bottom;
    AnalogEncoder rot;
    ToggleTelemetry telemetry;
    PIDController pid = new PIDController(0.01, 0.0, 0.02);
    boolean right=false;
    String direction = "";
    double error = 0.0;

    int rotationValueWindow = 7;
    double[] rotationValues = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    int rotationValueIndex = 0;
    double lastRotation;

    double lastMotorRotation;
    int noRotCount = 0;
    int noMotorRotCount = 0;

    private double motorRotation = 0.0; // motor based rotation
    private double uncorrectedMotorRotation = 0.0;

    double offset = 0;
    double maxPower = 1;

    public SwerveModule(Motor bottom, Motor top, AnalogEncoder rot, ToggleTelemetry telemetry) {
        this.top = top;
        this.bottom = bottom;
        this.rot = rot;
        this.telemetry = telemetry;
        lastRotation = rot.getDegrees();

        lastMotorRotation = motorRotation = rot.getDegrees();
        uncorrectedMotorRotation = rot.getDegrees();
        top.resetEncoder();
        bottom.resetEncoder();
    }
    /**
    *@param right false = left, true = right
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
        telemetry.addData("rawVoltage"+direction, rot.getRawVoltage()*100.0);

        //telemetry.addData("lastRotation"+direction, lastRotation);
        double change = Math.abs(AngleUnit.normalizeDegrees(rotation-lastRotation));
        //telemetry.addData("Change"+direction,change);
        telemetry.addData("milliseconds", System.currentTimeMillis() % 60000);
        telemetry.addData("RawRotation"+direction, rotation);
        double average = (rotation + lastRotation)/2.0;
        /*if (change<40.0) {
            telemetry.addData("average"+direction, "");
            lastRotation = rotation;
        }
        else if (compare(average, 180.0, 20.0)) {
            telemetry.addData("average" + direction, average);
            rotation = 360.0 - rotation;
            lastRotation = rotation;
        }
        else {
            telemetry.addData("average"+direction, average);
            rotation = lastRotation;
        }*/
        lastRotation = rotation;
        telemetry.addData("encRotation"+direction, rotation);
        motorRotation = getRotation();
        telemetry.addData("motorRotation"+direction, motorRotation);
        //telemetry.addData("encoderRotation"+direction,rot.getDegrees());

        double moveTo = AngleUnit.normalizeDegrees(heading-motorRotation);  // use rotation or motorRotation here
        if (!compare(moveTo,0.0,90.0)&&optimize) {
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
        if(compare(rot.getDegrees(),lastRotation,1.0))
            noRotCount++;
        else
            noRotCount = 0;
        if(noRotCount > 5 && noMotorRotCount > 5) {
            telemetry.addData("CorrectMotorRot"+direction, "100");
            //motorRotation = rot.getDegrees();
        }
        else
            telemetry.addData("CorrectMotorRot"+direction, "0");
        lastMotorRotation = motorRotation;
        telemetry.addData("uncorrectedeMotorRotation"+direction, uncorrectedMotorRotation);
        heading*=-1;

        //telemetry.addData("wheel"+direction, wheel);
        //telemetry.addData("headingpower"+direction, heading);
        wheel /= headingwheelratio;
        double topP = wheel-heading;
        double bottomP = wheel+heading;
        topP = Range.clip(topP, -1,1)/maxPower;
        bottomP = Range.clip(bottomP, -1, 1)/maxPower;



        double [] powers = {topP, bottomP};
        powers = normalize(powers, 1.0);
        //telemetry.addData("TopP"+direction, topP);
        //telemetry.addData("bottomP"+direction, bottomP);
        top.set(powers[0]);
        bottom.set(-powers[1]);
    }
    public double getRotation() {
        int topPosition = top.getCurrentPosition();
        int bottomPosition = bottom.getCurrentPosition();
        if(topPosition == 0 && bottomPosition == 0)
                noMotorRotCount++;
        else
            noMotorRotCount = 0;
        telemetry.addData("topTicks"+direction, topPosition);
        telemetry.addData("bottomTicks"+direction, bottomPosition);
        motorRotation += (topPosition*kTop-bottomPosition*kBottom)*rotationConstant;
        uncorrectedMotorRotation += (topPosition*kTop-bottomPosition*kBottom)*rotationConstant;
        uncorrectedMotorRotation =  ((uncorrectedMotorRotation%360.0)+360.0)%360.0;

        motorRotation = ((motorRotation%360.0)+360.0)%360.0;
        //motorRotation = AngleUnit.normalizeDegrees(motorRotation) ;
        top.resetEncoder();
        bottom.resetEncoder();
        return motorRotation;
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

    public void setOffset(double offset) {
        this.offset = offset;
        lastRotation = rot.getDegrees()+offset;

        lastMotorRotation = motorRotation = rot.getDegrees()+offset;
        uncorrectedMotorRotation = rot.getDegrees()+offset;
    }
    public void setMaxPower(double maxPower) {
        this.maxPower = maxPower;
    }

}

