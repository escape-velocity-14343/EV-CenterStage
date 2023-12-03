package org.firstinspires.ftc.teamcode.OpModes.auton;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.ArrayList;

@Config
@Autonomous(name="All 1+0 ඞ", group="Audience")
@Disabled
public class PurplePixelAllSides extends AutonTemplate {

    public static double forwarddist = 30;

    public static double PoskP = 0.1;
    public static double PoskD = 0;

    public static double tolerance = 1.5;
    public static double bucketStopDropAndRollPos = 0.2;
    public static int slideStopDropAndRollPos = 0;
    public static double timetodriveforward = 0.5;
    public static double purplePower = -0.22;
    public static double backdropYPosOffset = 2;


    public PIDController poscontroller;

    Pose2d odopose;
    public double voltageMultiplier = 1;
    public
    int
            cycleCount
            =
            0
            ;


    @Override
    public void runOpMode() {
        initAuton(false);
        voltageMultiplier = 12.5/voltageSensor.getVoltage();
        while (opModeInInit()) {
            update();
            updatePropReading();
            telemetry.update();
            odometry.reset();
        }
        propPortal.close();

        resetForStart();

        poscontroller = new PIDController(PoskP, 0, PoskD);
        double error = 1000;
        double state = 0;
        transferStates = states.NONE;
        timer.reset();
        boolean close = false;

        while (opModeIsActive()) {

            poscontroller.setPID(PoskP, 0, PoskD);
            poscontroller.setSetPoint(0);

            odopose = odometry.getPose();


            if (state==0) {
                double xTarget = forwarddist - (propPosition == 1 ? 5 : 4);
                double yTarget = 0;
                //double xmove = poscontroller.calculate(odopose.getX() - forwarddist + (propPosition == 1 ? 5 : 4));

                //double ymove = poscontroller.calculate(odopose.getY() -(propPosition-1)*2.5);
                //error = Math.pow(odopose.getX() - forwarddist + (propPosition == 1 ? 8 : 4), 2) + Math.pow(odopose.getY() + (propPosition == 1 ? 3 : 0), 2);

                swerve.headingLock(true, 0);
                if (pidToPosition(xTarget,yTarget)||timer.seconds()>2.0) {
                    timer.reset();
                    state=1;
                }

            } else if (state==1) {
                swerve.headingLock(true, Math.PI * (propPosition == 0 ? 1 : propPosition == 1 ? 0 : -1) / 2);
                swerve.driveFieldCentric(0,0,0);
                if (timer.seconds()>1) {
                    timer.reset();
                    state = 1.5;
                }

            } else if (state==1.5) {
                swerve.driveRobotCentric(0,-0.2,0);
                if (timer.seconds()>0.5) {
                    timer.reset();
                    state = 2;
                }
            }

            else if (state==2) {
                swerve.driveFieldCentric(0,0,0);
                swerve.headingLock(true, Math.PI * (propPosition == 0 ? 1 : propPosition == 1 ? 0 : -1) / 2);


                intake.intake(purplePower*voltageMultiplier);
                if (timer.seconds()>0.5) {
                    timer.reset();
                    state = 3;
                }
            }
            else if (state==3) {
                intake.stop();
                if (pidToPosition(12,-3)||timer.seconds()>2.0) {
                    timer.reset();
                    state = 4;
                    swerve.stop();
                }
                swerve.setAuton();


            }


            Log.println(Log.INFO, "Odomoetry", "x: " + odopose.getX() + ", y: " + odopose.getY() + ", rot: " + odopose.getRotation().getDegrees());

            update();
            telemetry.update();
        }


    }
    public boolean pidToPosition(double x, double y) {
        double xmove = -poscontroller.calculate(odopose.getX() - x);

        double ymove = -poscontroller.calculate(odopose.getY() - y);
        double error = Math.pow(odopose.getX() - x, 2) + Math.pow(odopose.getY() - y, 2);
        swerve.driveFieldCentric(ymove,-xmove, 0);
        return error<tolerance;
    }
}
