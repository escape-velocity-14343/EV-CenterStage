package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@TeleOp(name="TestTeleOp",group = "0")
public class TeleOp1 extends Robot {
    ElapsedTime time = new ElapsedTime();
    boolean previousShare = false;
    boolean previousCircle;
    int pixels = 0;

    @Override
    public void runOpMode() {
        initialize();
        waitForStart();
        while(opModeIsActive()) {
            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }
            pixels = bucket.pixelsIn();




            //swerve.goofyDrive(AngleUnit.normalizeDegrees(Math.toDegrees(Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x))+90), -gamepad1.right_stick_y);
            swerve.driveFieldCentric(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);
            if (gamepad1.options) {
                resetIMU();
            }
            if (gamepad1.right_trigger>0) {
                slides.moveSlides(gamepad1.right_trigger);
            }
            else if (gamepad1.left_trigger>0) {
                slides.moveSlides(-gamepad1.left_trigger);
            }
            else
                slides.moveSlides(0);

            if (gamepad1.left_bumper) {
                intake.intake(-1);
            } else if (gamepad1.right_bumper) {
                if (transferStates==states.INTAKE&&intakeProgress==intakePos.RETRACTED)
                    smartIntake(pixels);
                else
                    transferStates=states.INTAKE;


            } else
                intake.stop();
            if (gamepad1.square) {
                transferStates = states.INTAKE;
                intakeProgress = intakePos.EXTENDED;
            }
            if (gamepad1.dpad_up) {
                slides.tilt(outtakeTilt);
                transferStates = states.OUTTAKE;
            }
            if (gamepad1.dpad_left) {
                transferStates = states.INIT;
                //slides.tilt(0);
            }
            if (gamepad1.cross) {
                //slides.tilt(1);
                if (transferStates==states.INTAKE) {
                    bucket.latch();
                }
                transferStates = states.UNDERPASS;
            }

            if (gamepad1.share&&!previousShare) {
                swerve.toggleHeadingLock(90);
            }

            if (gamepad1.triangle) {

                transferStates = states.OUTTAKE;
                outtakeProgress = outtakePos.RETRACTED;
            }

            if (gamepad1.circle&&!previousCircle) {
                bucket.latchToggle();
            }
            update();
            telemetry.addData("slidesPos", slides.getPosition());
            telemetry.addData("time",time.milliseconds());
            telemetry.addData("pixels", pixels);

            time.reset();
            previousShare = gamepad1.share;
            previousCircle = gamepad1.circle;

        }
    }
}
