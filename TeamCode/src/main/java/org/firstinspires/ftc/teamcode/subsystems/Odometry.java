package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drivers.ToggleTelemetry;

@Config
public class Odometry {
    ToggleTelemetry telemetry;
    Motor xEnc,yEnc;
    DcMotor xE, yE;
    double fx,fy,dx,dy,heading,dh; //heading radians
    double dxTotal, dyTotal;
    double lastHeading=0, lastx=0, lasty=0;
    Canvas field = new Canvas();
    TelemetryPacket packet = new TelemetryPacket();
    double targetx=-10000, targety=-10000;

    // TODO: Empirically tune these
    public static double TICKS_PER_INCH = 0;
    public static double X_ROTATE = 0;
    public static double Y_ROTATE = 0;
    public static boolean reverseX = false;
    public static boolean reverseY = true;


    public Odometry (HardwareMap hMap, ToggleTelemetry telemetry) {
        this.telemetry = telemetry;
        xE = hMap.dcMotor.get("intake");
        xE.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        xE.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        yE = hMap.dcMotor.get("xEnc");
        yE.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        yE.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        xEnc = new Motor(hMap,"intake");
        xEnc.resetEncoder();
        yEnc = new Motor(hMap,"xEnc");
        yEnc.resetEncoder();
        reset();



    }
    public void setTarget(double x, double y) {
        targetx = x;
        targety = y;
    }
    public void update(double botHeading) {

        heading = botHeading;
        dh = AngleUnit.normalizeRadians(heading-lastHeading) ;
        dx = (xEnc.getCurrentPosition()-lastx)/TICKS_PER_INCH-dh*X_ROTATE;
        dy = (-yEnc.getCurrentPosition()-lasty)/TICKS_PER_INCH-dh*Y_ROTATE;
        if (reverseX) {
            dx*=-1;
        }
        if (reverseY) {
            dy*=-1;
        }
        dxTotal += dx;
        dyTotal +=dy;
        fx += dx * Math.cos(botHeading) - dy * Math.sin(botHeading);
        fy += dx * Math.sin(botHeading) + dy * Math.cos(botHeading);
        lastHeading = heading;
        lastx = xEnc.getCurrentPosition();
        lasty = -yEnc.getCurrentPosition();
        TelemetryPacket packet = new TelemetryPacket();
        field = packet.fieldOverlay();
        int robotRadius = 8;
        field.strokeCircle(fx, fy, robotRadius);
        double arrowX = new Rotation2d(heading).getCos() * robotRadius, arrowY = new Rotation2d(heading).getSin() * robotRadius;
        double x1 = fx, y1 = fy;
        double x2 = fx + arrowX, y2 = fy+ arrowY;
        field.strokeLine(x1, y1, x2, y2);
        field.setFill("yellow");
        field.setStroke("yellow");
        field.fillCircle(targetx, targety, 2);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);


        telemetry.addData("xenc", xEnc.getCurrentPosition());
        telemetry.addData("yenc", yEnc.getCurrentPosition());
        telemetry.addData("dx", getDx());
        telemetry.addData("dy",getDy());

        telemetry.addData("pose x", getPose().getX());
        telemetry.addData("pose y", getPose().getY());
        telemetry.addData("total dx", dxTotal);
        telemetry.addData("total dy", dyTotal);

        //telemetry.put("time", System.currentTimeMillis());

    }
    public void reset() {
        xE.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        xE.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        yE.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        yE.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        dx=0;
        dy=0;
        dh=0;
        dxTotal=0;
        dyTotal=0;
        fx=0;
        fy=0;
        lastx = 0;
        lasty = 0;

    }
    public void reset(double x, double y) {
        xE.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        xE.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        yE.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        yE.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        dx=0;
        dy=0;
        dh=0;
        dxTotal=0;
        dyTotal=0;
        fx=x;
        fy=y;
        lastx = 0;
        lasty = 0;
    }
    public double getDx() {
        return dx;
    }
    public double getDy() {
        return dy;
    }
    public double getVelocity() {
        return Math.sqrt(Math.pow(dx, 2) + Math.pow(dy, 2));
    }

    private double moveTol = 0.01;
    public void setTol(double tol) {
        this.moveTol = tol;
    }
    public boolean isMoving() {
        double velo = getVelocity();
        return !(-moveTol < velo && velo < moveTol);
    }
    public Pose2d getPose() {
        return new Pose2d(fx,fy,new Rotation2d(heading));
    }
}
