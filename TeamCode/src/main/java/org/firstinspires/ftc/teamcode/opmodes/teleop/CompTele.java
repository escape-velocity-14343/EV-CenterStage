package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Robot;


@TeleOp(name="Comp TeleOp")
public class CompTele extends Robot {

    boolean lastOptions2 = false;
    boolean lastOptions = false;

    @Override
    public void runOpMode() {
        initialize();
        waitForStart();
        while (opModeIsActive()) {
            update();

            swerve.driveFieldCentric(gamepad1c.left_stick_x, -gamepad1c.left_stick_y, gamepad1c.right_stick_x);

            // outtake
            if (inOuttake() && isDone()) {
                if (gamepad1c.left_trigger > 0.7) {
                    setOuttake(getArmDistance(), getArmHeight() - gamepad1c.left_trigger * (loopNanos / 1e8));
                } else if (gamepad1c.right_trigger > 0.7) {
                    setOuttake(getArmDistance(), getArmHeight() + gamepad1c.right_trigger * (loopNanos / 1e8));
                }

                if (gamepad1c.square) {
                    bucket.setLeftLatch(false);
                }
                if (gamepad1c.circle) {
                    bucket.setRightLatch(false);
                }
                if (gamepad2c.square) {
                    bucket.setLeftLatch(true);
                }
                if (gamepad2c.circle) {
                    bucket.setRightLatch(true);
                }
                if (gamepad2c.left_trigger > 0.7) {
                    setOuttake(getArmDistance() - gamepad2c.left_trigger * (loopNanos / 1e8), getArmHeight());
                } else if (gamepad2c.right_trigger > 0.7) {
                    setOuttake(getArmDistance() + gamepad2c.right_trigger * (loopNanos / 1e8), getArmHeight());
                }
                if (gamepad2c.triangle) {
                    bucket.dropFromStack();
                }

            }
            // intake
            if (inIntake() && isDone()) {
                if (gamepad1c.left_trigger > 0.7) {
                    arm.moveSlides(gamepad1c.left_trigger);
                } else if (gamepad1c.right_trigger > 0.7) {
                    arm.moveSlides(-gamepad1c.right_trigger);
                }

                if (gamepad1c.square) {
                    bucket.setLeftLatch(!bucket.leftIsLatched());
                }
                if (gamepad1c.circle) {
                    bucket.setRightLatch(!bucket.rightIsLatched());
                }
            }

            // fsm transitions
            if (gamepad1c.triangle) {
                outtake();
                setOuttake(10, getArmHeight());
            }
            if (gamepad1c.cross) {
                outtake();
                setOuttake(24, getArmHeight());
            }
            if (gamepad1c.right_bumper) {
                intake();
                setIntake(180);
            }
            if (gamepad1c.left_bumper) {
                intake();
                setIntake(175);
            }

            // overrides
            if (gamepad2c.options && !lastOptions2) {
                bucket.disableAutoLatch = true;
            }
            if (gamepad1c.options && !lastOptions) {
                imu.resetYaw();
            }

            lastOptions2 = gamepad2c.options;
            lastOptions = gamepad1c.options;
        }
    }
}
