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
import org.firstinspires.ftc.teamcode.drivers.DashboardPacketTelemetry;
import org.firstinspires.ftc.teamcode.drivers.ToggleTelemetry;

@Config
public class Odometry {
    ToggleTelemetry telemetry;
    DashboardPacketTelemetry fieldtelem;
    DcMotor xE, yE;
    double fx,fy,dx,dy,heading,dh; //heading radians
    double dxTotal, dyTotal;
    double lastHeading=0, lastx=0, lasty=0;
    Canvas field = new Canvas();
    TelemetryPacket packet = new TelemetryPacket();
    double targetx=-10000, targety=-10000;

    // TODO: Empirically tune these
    public static double TICKS_PER_INCH = 936;
    public static double X_ROTATE = 4.5;
    public static double Y_ROTATE = 6.11536953336;
    public static boolean reverseX = false;
    public static boolean reverseY = true;

    private long lastLoopNanos = 1;


    public Odometry (HardwareMap hMap, ToggleTelemetry telemetry) {
        this.telemetry = telemetry;
        //this.fieldtelem = fieldtelem;
        xE = hMap.dcMotor.get("bottomright");
        xE.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        xE.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        yE = hMap.dcMotor.get("topright");
        yE.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        yE.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while(fx!=0)
            reset();



    }
    public void setTarget(double x, double y) {
        targetx = x;
        targety = y;
    }
    public void update(double botHeading, long loopNanos) {

        lastLoopNanos = loopNanos;

        heading = botHeading;
        dh = AngleUnit.normalizeRadians(heading-lastHeading) ;
        dx = (xE.getCurrentPosition()-lastx)/TICKS_PER_INCH-dh*X_ROTATE;
        dy = (-yE.getCurrentPosition()-lasty)/TICKS_PER_INCH-dh*Y_ROTATE;
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
        lastx = xE.getCurrentPosition();
        lasty = -yE.getCurrentPosition();
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


        telemetry.addData("xenc", xE.getCurrentPosition());
        telemetry.addData("yenc", yE.getCurrentPosition());
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
        lastHeading = 0;
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
        lastHeading = 0;
        lastx = 0;
        lasty = 0;
    }

    public void reset(double x, double y, double heading) {
        reset(x, y);
        lastHeading = heading;
    }
    public double getDx() {
        return dx;
    }
    public double getDy() {
        return dy;
    }

    /**
     * @return Inches per second. Time is calculated based on last odometry update call.
     */
    public double getVelocity() {
        return Math.sqrt(Math.pow(dx, 2) + Math.pow(dy, 2))/(lastLoopNanos/1e9);
    }

    public int[] getTicks() {
        return new int[]{xE.getCurrentPosition(), yE.getCurrentPosition()};
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
