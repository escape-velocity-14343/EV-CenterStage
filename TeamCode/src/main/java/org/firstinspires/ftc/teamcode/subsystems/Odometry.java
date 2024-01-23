package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drivers.DashboardPacketTelemetry;
import org.firstinspires.ftc.teamcode.drivers.ToggleTelemetry;

@Config
public class Odometry {
    ToggleTelemetry telemetry;
    DashboardPacketTelemetry fieldtelem;
    DcMotor xE, xE2, yE;
    IMU imu;
    double fx,fy,dx, dx2, dc, dy,heading,dh; //heading radians
    double dxTotal, dyTotal;
    double lastHeading=0, lastx=0, lasty=0, lastx2=0;
    Canvas field = new Canvas();
    TelemetryPacket packet = new TelemetryPacket();
    double targetx=-10000, targety=-10000;
    public double targetx2=-10000, targety2=-10000;

    // TODO: Empirically tune these
    public static double TICKS_PER_INCH = 936;
    public static double X_ROTATE = 3.75;
    public static double Y_ROTATE = 6.11536953336;
    public static boolean reverseX = false;
    public static boolean reverseY = false;
    public static boolean reverseHeading = false;
    public static boolean usePersistent = false;
    public static double persistentSeconds = 1;
    private ElapsedTime persistentTimer = new ElapsedTime();
    public static double trackwidth = 10;

    private long lastLoopNanos = 1;
    private boolean hasRun = false;



    public Odometry (HardwareMap hMap, ToggleTelemetry telemetry) {
        this.telemetry = telemetry;
        //this.fieldtelem = fieldtelem;
        xE = hMap.dcMotor.get("bottomright");
        xE.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        xE.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        yE = hMap.dcMotor.get("topright");
        yE.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        yE.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        imu = hMap.get(IMU.class, "imu 1");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
        imu.initialize(parameters);
        initialIMUreading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        while(fx!=0)
            reset();



    }
    public void setTarget(double x, double y) {
        targetx = x;
        targety = y;
    }

    /**
     * 2 wheel update function
     */
    public void update(double botHeading, long loopNanos) {

        lastLoopNanos = loopNanos;

        heading = botHeading;
        dh = AngleUnit.normalizeRadians(heading-lastHeading) ;
        // fix inexact angles
        if (!hasRun) {
            dh = 0;
            hasRun = true;
        }
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
        field.strokeLine(x1, y1, x1+targetx2, y1+targety2);
        if (!Robot.useDashTelemetry) {
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }


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

    /**
     * 3 wheel update function
     * @param loopNanos
     */
    public void update(long loopNanos) {
        lastLoopNanos = loopNanos;

        int xEnc = xE.getCurrentPosition();
        int xEnc2 = xE2.getCurrentPosition();
        int yEnc = yE.getCurrentPosition();

        dx = (xEnc-lastx)/TICKS_PER_INCH;
        dx2 = (xEnc2-lastx2)/TICKS_PER_INCH;


        dh = (dx - dx2) / trackwidth;



        dc = (dx2 + dx) / 2;

        dy = (yEnc-lasty)/TICKS_PER_INCH - Y_ROTATE*dh;

        if (reverseX) {
            dx*=-1;
            dx2*=-1;
        }
        if (reverseY) {
            dy*=-1;
        }
        if (reverseHeading) {
            dh *= -1;
        }
        // TODO: make this not as bad lol
        dx = dc;

        dxTotal += dx;
        dyTotal +=dy;
        heading = lastHeading + dh;
        fx += dx * Math.cos(heading) - dy * Math.sin(heading);
        fy += dx * Math.sin(heading) + dy * Math.cos(heading);
        lastHeading = heading;
        lastx = xEnc;
        lastx2 = xEnc2;
        lasty = yEnc;
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
        field.strokeLine(x1, y1, x1+targetx2, y1+targety2);
        if (!Robot.useDashTelemetry) {
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }

        if (persistentTimer.seconds() > persistentSeconds && usePersistent) {
            lastHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) - initialIMUreading + initialHeading;
        }


        telemetry.addData("xenc", xE.getCurrentPosition());
        telemetry.addData("yenc", yE.getCurrentPosition());
        telemetry.addData("dx", getDx());
        telemetry.addData("dy",getDy());

        telemetry.addData("pose x", getPose().getX());
        telemetry.addData("pose y", getPose().getY());
        telemetry.addData("total dx", dxTotal);
        telemetry.addData("total dy", dyTotal);
    }

    public void reset() {
        xE.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        xE.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        yE.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        yE.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        dx=0;
        dx2=0;
        dy=0;
        dh=0;
        dxTotal=0;
        dyTotal=0;
        fx=0;
        fy=0;
        lastHeading = 0;
        lastx = 0;
        lastx2 = 0;
        lasty = 0;

    }
    public void reset(double x, double y) {
        xE.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        xE.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        yE.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        yE.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        dx=0;
        dx2=0;
        dy=0;
        dh=0;
        dxTotal=0;
        dyTotal=0;
        fx=x;
        fy=y;
        lastHeading = 0;
        lastx = 0;
        lastx2 = 0;
        lasty = 0;
    }

    public void resetYaw() {
        imu.resetYaw();
    }

    private double initialHeading = 0;
    private double initialIMUreading = 0;

    public void reset(double x, double y, double heading) {
        reset(x, y);
        lastHeading = heading;
        initialHeading = heading;
        initialIMUreading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
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

    public Pose2d getVelocityPose() {
        return new Pose2d((dx * Math.cos(heading) - dy * Math.sin(heading))/(lastLoopNanos/1e9), (dx * Math.sin(heading) + dy * Math.cos(heading))/(lastLoopNanos/1e9), new Rotation2d(dh/(lastLoopNanos/1e9)));
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
    public double getAnglularVelocity() {
        return dh;
    }
    public Pose2d getPose() {
        return new Pose2d(fx,fy,new Rotation2d(heading));
    }
}
