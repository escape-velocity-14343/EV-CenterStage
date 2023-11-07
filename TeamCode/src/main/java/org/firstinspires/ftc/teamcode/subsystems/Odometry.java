package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Odometry {
    Telemetry telemetry;
    Motor xEnc,yEnc;
    double fx,fy,dx,dy,heading,dh; //heading radians
    double dxTotal, dyTotal;
    double lastHeading=0, lastx=0, lasty=0;
    Canvas field = new Canvas();
    TelemetryPacket packet = new TelemetryPacket();


    public Odometry (HardwareMap hMap,Telemetry telemetry) {
        this.telemetry = telemetry;
        xEnc = new Motor(hMap,"intake");
        xEnc.resetEncoder();
        yEnc = new Motor(hMap,"xEnc");
        yEnc.resetEncoder();


    }
    public void update(double botHeading) {

        heading = botHeading;
        dh = AngleUnit.normalizeRadians(heading-lastHeading) ;
        dx = (xEnc.getCurrentPosition()-lastx)/Robot.TICKS_PER_INCH-dh*Robot.X_ROTATE;
        dy = (-yEnc.getCurrentPosition()-lasty)/Robot.TICKS_PER_INCH-dh*Robot.Y_ROTATE;
        if (Robot.reverseX) {
            dx*=-1;
        }
        if (Robot.reverseY) {
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
        double x1 = fx + arrowX  / 2, y1 = fy + arrowY / 2;
        double x2 = fx + arrowX, y2 = fy+ arrowY;
        field.strokeLine(x1, y1, x2, y2);
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
        dx=0;
        dy=0;
        dh=0;
        dxTotal=0;
        dyTotal=0;
        fx=0;
        fy=0;

    }
    public void reset(double x, double y) {
        dx=0;
        dy=0;
        dh=0;
        dxTotal=0;
        dyTotal=0;
        fx=x;
        fy=y;
    }
    public double getDx() {
        return dx;
    }
    public double getDy() {
        return dy;
    }
    public Pose2d getPose() {
        return new Pose2d(fx,fy,new Rotation2d(heading));
    }
}
