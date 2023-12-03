package org.firstinspires.ftc.teamcode.OpModes.teleop;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@TeleOp(name="Comp Teleop",group = "0")
public class TeleOp1 extends Robot {

    boolean previousShare = false;
    boolean previousShare2 = false;
    boolean previousCircle;
    boolean toGooberAlign = false;


    @Override
    public void runOpMode() {
        initialize();
        headingOffset = 0;
        waitForStart();
        Gamepad gamepad1copy = new Gamepad();
        Gamepad gamepad2copy = new Gamepad();
        while(opModeIsActive()) {
            gamepad1copy.copy(gamepad1);
            gamepad2copy.copy(gamepad2);

            //swerve.goofyDrive(AngleUnit.normalizeDegrees(Math.toDegrees(Math.atan2(-gamepad1copy.left_stick_y, gamepad1copy.left_stick_x))+90), -gamepad1copy.right_stick_y);
            swerve.driveFieldCentric(gamepad1copy.left_stick_x, -gamepad1copy.left_stick_y, gamepad1copy.right_stick_x*2);
            if (gamepad1copy.options) {
                resetIMU();
            }
            if (transferStates == states.OUTTAKE||transferStates == states.EXTENDO||transferStates == states.DROP||transferStates == states.HANG) {
                if (gamepad1copy.right_trigger>0) {
                    slides.moveSlides(gamepad1copy.right_trigger);
                }
                else if (gamepad1copy.left_trigger>0) {
                    slides.moveSlides(-gamepad1copy.left_trigger);
                }
                else
                    slides.moveSlides(0);
            }

            if (gamepad1copy.left_bumper) {
                intake.intake(-reverseIntakePower);
                lastIntake = false;
            } else if (gamepad1copy.right_bumper) {
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
                intake.armUp();
                intake.stop();
                lastIntake = false;
            }
            /*if (gamepad1copy.square) {
                transferStates = states.INTAKE;
                intakeProgress = intakePos.EXTENDED;
            }*/
            if (gamepad1copy.cross) {
                //slides.tilt(1);
                if (transferStates==states.INTAKE) {
                    bucket.latch();
                }
                underpass();
            }

            if (gamepad2copy.dpad_up||gamepad1copy.dpad_up) {
                drone.setPosition(dronePos);
            }

            if (gamepad2copy.dpad_down) {
                drone.setPosition(1-dronePos);
            }

            if (gamepad1copy.touchpad) {
                toGooberAlign = true;
            }
            if (!gamepad1copy.touchpad_finger_1) {
                toGooberAlign = false;
            }

            if (toGooberAlign) {
                if (gooberAlign(gamepad1copy)) {
                    gamepad1.rumble(50);
                }
            }

            if (gamepad2copy.dpad_right||gamepad1copy.dpad_right) {
                dropAll();
            }

            if (gamepad1copy.triangle) {
                outtake();
            }
            if (gamepad1copy.square) {
                extendo();
            }

            if (gamepad1copy.circle&&!previousCircle) {
                if (outtakeProgress==outtakePos.EXTENDED) {
                    drop();
                }
                else if (extendoProgress==extendoProgress.EXTENDED) {
                    drop();
                }
                else {
                    Log.println(Log.WARN, "Bucket", "toggle toggle");
                    bucket.latchToggle();
                }
            }
            if (gamepad1copy.share&&!previousShare) {
                swerve.toggleHeadingLock(90*(Robot.headingLockIsFlipped?-1:1));
            }
            if (gamepad2copy.options) {
                transferStates = states.NONE;
                gamepad1copy.rumble(1000);
            }
            if (gamepad1copy.dpad_down) {
                hang();
                hangPID();
            }
            if (transferStates==states.NONE) {
                if (gamepad2copy.right_trigger>0) {
                    slides.moveSlides(gamepad2copy.right_trigger);
                    //slides.motor1.set(gamepad2copy.right_trigger);
                }
                else if (gamepad2copy.left_trigger>0) {
                    slides.moveSlides(-gamepad2copy.left_trigger);
                    //slides.motor1.set(-gamepad2copy.left_trigger);
                }
                else
                    slides.moveSlides(0);

                if (gamepad2copy.cross) {
                    bucket.intake();
                }
                if (gamepad2copy.circle) {
                    bucket.latchToggle();
                }
                if (gamepad2copy.triangle) {
                    bucket.outtake();
                }
                if (gamepad2copy.left_bumper) {
                    intake.intake(-1);
                }
                if (gamepad2copy.right_bumper) {
                    intake.intake(1);
                }


                if (gamepad2copy.dpad_left) {
                    bucket.setPosition(bucketInitPos);
                    slides.reset();
                }
            }
            if (getRuntime()>90 && getRuntime()<91) {
                gamepad1copy.rumbleBlips(3);
            }
            if (gamepad2copy.share && !previousShare2) {
                Robot.headingLockIsFlipped = !Robot.headingLockIsFlipped;
                swerve.updateHeadingLock(Math.PI/2 * (Robot.headingLockIsFlipped?-1:1));
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
            telemetry.addData("hub 0 amps", allHubs.get(0).getCurrent(CurrentUnit.AMPS));
            telemetry.addData("hub 1 amps", allHubs.get(1).getCurrent(CurrentUnit.AMPS));
            telemetry.addData("voltage", allHubs.get(1).getInputVoltage(VoltageUnit.VOLTS));
            telemetry.update();

            time.reset();
            previousShare = gamepad1copy.share;
            previousCircle = gamepad1copy.circle;
            previousShare2 = gamepad2copy.share;


        }
    }
}
