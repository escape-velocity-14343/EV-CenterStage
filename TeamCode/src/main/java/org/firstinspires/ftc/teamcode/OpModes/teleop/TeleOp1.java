package org.firstinspires.ftc.teamcode.OpModes.teleop;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@TeleOp(name="Comp Teleop",group = "0")
public class TeleOp1 extends Robot {

    boolean previousShare = false;
    boolean previousShare2 = false;
    boolean previousCircle;


    @Override
    public void runOpMode() {
        initialize();
        waitForStart();
        while(opModeIsActive()) {

            //swerve.goofyDrive(AngleUnit.normalizeDegrees(Math.toDegrees(Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x))+90), -gamepad1.right_stick_y);
            swerve.driveFieldCentric(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);
            if (gamepad1.options) {
                resetIMU();
            }
            if (transferStates == states.OUTTAKE||transferStates == states.EXTENDO||transferStates == states.DROP) {
                if (gamepad1.right_trigger>0) {
                    slides.moveSlides(gamepad1.right_trigger);
                }
                else if (gamepad1.left_trigger>0) {
                    slides.moveSlides(-gamepad1.left_trigger);
                }
                else
                    slides.moveSlides(0);
            }

            if (gamepad1.left_bumper) {
                intake.intake(-reverseIntakePower);
                lastIntake = false;
            } else if (gamepad1.right_bumper) {
                if (transferStates==states.INTAKE&&intakeProgress==intakePos.RETRACTED) {
                    smartIntake();
                    lastIntake = true;
                }
                else if (transferStates==states.INTAKE){
                    lastIntake = false;
                }
                else {
                    lastIntake = false;
                    intake();
                }


            } else {
                intake.stop();
                lastIntake = false;
            }
            /*if (gamepad1.square) {
                transferStates = states.INTAKE;
                intakeProgress = intakePos.EXTENDED;
            }*/
            if (gamepad1.cross) {
                //slides.tilt(1);
                if (transferStates==states.INTAKE) {
                    bucket.latch();
                }
                transferStates = states.UNDERPASS;
                tiltProgess = tiltPos.WORKING;
            }

            if (gamepad2.dpad_up&&gamepad1.dpad_up) {
                drone.setPosition(dronePos);
            }

            if (gamepad2.dpad_down) {
                drone.setPosition(1-dronePos);
            }


            if (gamepad1.triangle) {
                outtake();
            }
            if (gamepad1.square) {
                extendo();
            }

            if (gamepad1.circle&&!previousCircle) {
                if (outtakeProgress==outtakePos.EXTENDED) {
                    drop();
                }
                else if (extendoProgress==outtakePos.EXTENDED) {
                    drop();
                }
                else {
                    telemetry.addLine("toggle toggle");
                    bucket.latchToggle();
                }
            }
            if (gamepad1.share&&!previousShare) {
                swerve.toggleHeadingLock(90*(Robot.headingLockIsFlipped?-1:1));
            }
            if (gamepad2.options) {
                transferStates = states.NONE;
                gamepad1.rumble(1000);
            }
            if (transferStates==states.NONE) {
                if (gamepad2.right_trigger>0) {
                    slides.moveSlides(gamepad2.right_trigger);
                }
                else if (gamepad2.left_trigger>0) {
                    slides.moveSlides(-gamepad2.left_trigger);
                }
                else
                    slides.moveSlides(0);

                if (gamepad2.cross) {
                    bucket.intake();
                }
                if (gamepad2.circle) {
                    bucket.latchToggle();
                }
                if (gamepad2.triangle) {
                    bucket.outtake();
                }
                if (gamepad2.left_bumper) {
                    intake.intake(-1);
                }
                if (gamepad2.right_bumper) {
                    intake.intake(1);
                }
            }
            if (getRuntime()>90 && getRuntime()<91) {
                gamepad1.rumbleBlips(3);
            }
            if (gamepad2.share && !previousShare2) {
                Robot.headingLockIsFlipped = !Robot.headingLockIsFlipped;
                Log.println(Log.WARN, "HeadingLock", "Heading lock has been flipped to " + (Robot.headingLockIsFlipped?"flipped":"normal") + ".");
            }
            update();
            telemetry.addData("latch pos", bucket.latchPos());
            telemetry.addData("slidesPos", slides.getPosition());
            telemetry.addData("time",time.milliseconds());
            telemetry.addData("pixels", pixels);
            telemetry.addData("intake current draw", intake.getCurrentDraw());
            telemetry.addData("lockforever", lockForever);
            telemetry.addData("lastintake", lastIntake);
            telemetry.update();

            time.reset();
            previousShare = gamepad1.share;
            previousCircle = gamepad1.circle;
            previousShare2 = gamepad2.share;


        }
    }
}
