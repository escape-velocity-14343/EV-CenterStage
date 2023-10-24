package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.subsystems.SwerveModule.compare;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drivers.AnalogEncoder;


@Config
public class SwerveController {
    public static double p = 0.001;
    public static double i = 0.0;
    public static double d = 0.0;

    public static double loffset = 0.0;
    public static double roffset = 0.0;

    public static double headingwheelratio = 1.0;
    double lt,rt;
    double jx;
    double jy;
    double rot;
    double lx,ly,rx,ry;
    double[] powers = {0,0,0,0};

    Translation2d translation = new Translation2d();
    SwerveModule left, right;
    IMU imu;
    Telemetry telemetry;
    VoltageSensor voltageSensor;
    public SwerveController(HardwareMap hMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        left = new SwerveModule(new Motor(hMap,"bottomleft"),new Motor(hMap,"topleft"),new AnalogEncoder(hMap,"leftrot"),telemetry);
        left.rot.setOffset(loffset);
        left.setSide(false);
        right = new SwerveModule(new Motor(hMap,"bottomright"),new Motor(hMap,"topright"),new AnalogEncoder(hMap,"rightrot"),telemetry);
        right.rot.setOffset(roffset);
        right.setSide(true);

        imu = hMap.get(IMU.class,"imu 1");
        voltageSensor = hMap.voltageSensor.iterator().next();
    }
    private void drive(double x, double y, double rot, double botHeading) {
        left.setPid(p,i,d);
        right.setPid(p,i,d);
        translation = new Translation2d(jx,jy);
        jx = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        jy = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        powers = new double[]{jx, jy + rot, jx, jy - rot};

        lx = normalize(powers, 1)[0];
        ly = normalize(powers, 1)[1];
        rx = normalize(powers, 1)[2];
        ry = normalize(powers, 1)[3];
        lt = Math.atan2(ly,lx);
        rt = Math.atan2(ry,rx);
        lt = Math.toDegrees(lt);
        rt = Math.toDegrees(rt);
        if (!compare(Math.abs(x)+Math.abs(y)+Math.abs(rot),0.0,0.001)) {
            left.podPidXY(lx,ly);
            right.podPidXY(rx,ry);
        }
        else {
            left.podPid(0.0, left.rot.getDegrees());
            right.podPid(0.0, right.rot.getDegrees());
        }
        //telemetry.addData("voltage", voltageSensor.getVoltage());
        //telemetry.addData("Left encoder", left.rot.getDegrees());
        //telemetry.addData("Right encoder", right.rot.getDegrees());
        //telemetry.addData("yaw",imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        telemetry.update();
    }
    public void goofyDrive(double direction, double power) {
        left.setPid(p,i,d);
        right.setPid(p,i,d);
        if (power>0.01) {
            left.podPid(power,direction);
            right.podPid(power,direction);
        }
        else {
            left.podPid(0.0, 90);
            right.podPid(0.0, 90);
        }

    }
    public void driveFieldCentric(double x, double y, double rot) {
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        drive(x,y,rot,botHeading);
    }
    public void driveRobotCentric(double x, double y, double rot) {
        drive(x,y,rot,0);
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
}
