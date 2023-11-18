package org.firstinspires.ftc.teamcode.OpModes.auton;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.ArrayList;

@Config
@Autonomous(name="Blue Backdrop 2+4 ඞ", group="Audience")
public class BlueBackdropBothVisionNew extends AutonTemplate {

    public static double forwarddist = 30;

    public static double PoskP = 0.1;
    public static double PoskD = 0;

    public static double tolerance = 1.5;
    public static double bucketStopDropAndRollPos = 0.2;
    public static int slideStopDropAndRollPos = 0;
    public static double timetodriveforward = 0.5;
    public static double purplePower = -0.25;
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
        initAuton();
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
                    initAprilTag();
                }
                swerve.setAuton();


            }
            else if (state==4) {
                if (pidToPosition(12,-24)||timer.seconds()>1.0) {
                    timer.reset();
                    state = 5;
                }
                swerve.setAuton();
            }
            else if (state==5) {
                if (pidToPosition(26-(propPosition-1)*6,-32)||timer.seconds()>2.0) {
                    timer.reset();
                    state = 6;
                }
                slides.tilt(outtakeTilt);
                swerve.headingLock(true, Math.PI/2);
            }
            else if (state==6) {
                timer.reset();

                outtake();
                swerve.headingLock(true, Math.PI/2);
                swerve.driveFieldCentric(0,0,0);
                state=7;
            }
            else if (state==7) {
                slides.pidSlides(slideOuttakeEncPos+50+250*cycleCount);
                swerve.headingLock(true, Math.PI/2);
                double ypos = -36;
                ArrayList<AprilTagDetection> detections = aprilTag.getDetections();
                if (detections.size() > 0) {
                    double sum = 0;
                    for (AprilTagDetection detection : detections) {
                        sum += detection.ftcPose.y;
                    }
                    sum /= detections.size();
                    ypos = odopose.getY()-sum+backdropYPosOffset;
                }
                if (pidToPosition(26-(propPosition-1)*6,ypos)||timer.seconds()>0.5) {
                    timer.reset();
                    state = 8;
                }
            }
            else if (state==8) {
                slides.pidSlides(slideOuttakeEncPos+50+250*cycleCount);
                swerve.headingLock(true, Math.PI/2);
                swerve.driveFieldCentric(0,0,0);
                if (done) {
                    timer.reset();
                    state = 9;
                }
            }
            else if (state == 9) {
                slides.pidSlides(slideOuttakeEncPos+50+250*cycleCount);
                swerve.headingLock(true, Math.PI/2);
                swerve.driveFieldCentric(0,0,0);
                if (timer.seconds()>0.2) {
                    timer.reset();
                    state = 10;
                }
            }
            else if (state == 10) {
                drop();
                timer.reset();
                state = 11;
            }
            else if (state==11) {
                if (timer.seconds()>0.3) {
                    timer.reset();
                    state = 13;
                }
            }
            else if (state==13) {
                swerve.headingLock(true, Math.PI/2);
                if (pidToPosition(50,-12)||timer.seconds()>0.5) {
                    underpass();
                    timer.reset();
                    state = 14;
                }
            }
            else if (state==14) {
                swerve.headingLock(true, Math.PI/2);
                if (pidToPosition(50,-12)||timer.seconds()>0.5) {
                    if (done&&timer.seconds()>0.5) {
                        timer.reset();
                        state = 15;
                    }
                }
            }
            else if (state==15) {
                swerve.headingLock(true, Math.PI/2);
                if (pidToPosition(50,48)||timer.seconds()>3) {
                    timer.reset();
                    state = 16;
                    intake();
                    Log.println(Log.INFO,"auton", "Time: "+getRuntime());
                }
            }
            else if (state==16) {

                if (pidToPosition(52,66)||timer.seconds()>0.2) {
                    if (done) {
                        timer.reset();
                        state = 17;
                    }
                }
            }
            else if (state==17) {

                if (pidToPosition(52,63)||timer.seconds()>0.4) {
                    timer.reset();
                    state=17.5;
                }
            }
            else if (state==17.5) {
                swerve.headingLock(true, Math.PI/2);
                swerve.driveFieldCentric(-0.4,0,0);
                if (smartIntake()) {
                    timer.reset();
                    underpass();
                    state=18;
                }
                if (timer.seconds()>2) {
                    timer.reset();
                    state = 16;
                }
            }
            else if (state==18) {
                swerve.headingLock(true, Math.PI/2);
                intake.intake(-0.5);
                if (pidToPosition(50,48)||timer.seconds()>1) {
                    if (timer.seconds()>1) {
                    timer.reset();
                    state = 19;
                    }
                }
            }
            //
            else if (state==19) {
                intake.stop();
                swerve.headingLock(true, Math.PI/2);
                if (pidToPosition(50,-12)||timer.seconds()>2) {
                    timer.reset();
                    state = 20;
                    propPosition=0;
                }
            }
            //i dont konw
            else if (state==20) {
                swerve.headingLock(true, Math.PI/2);
                if (pidToPosition(30,-32)||timer.seconds()>1) {
                    timer.reset();
                    outtake();
                    cycleCount++;
                    if (cycleCount<3) {
                        state = 6;
                    }
                    else {
                        state=21;
                    }
                    swerve.stop();

                }
            }
            //parl
            else if (state==21) {
                swerve.headingLock(true, Math.PI/2);
                pidToPosition(50,-30);
            }


            /*else if (timer.seconds() < timetodriveforward + 3) {
                swerve.driveRobotCentric(0, 0.3, 0);
            } else if (timer.seconds() < timetodriveforward + 3.5) {
                swerve.stop();
                slides.pidSlides(1000);
            } else if (timer.seconds() < 4.3) {
                bucket.setPosition(bucketStopDropAndRollPos);
            } else if (timer.seconds() < 4.6) {
                slides.pidSlides(slideStopDropAndRollPos);
            } else if (timer.seconds() < 5&&!done) {
                drop();
            } else if (timer.seconds() < 5.3) {
                bucket.latch();
            } else if (timer.seconds() < 6) {
                slides.pidSlides(1000);
            } else {
                swerve.stop();
            }*/

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
