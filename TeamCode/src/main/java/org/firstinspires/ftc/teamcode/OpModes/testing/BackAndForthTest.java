package org.firstinspires.ftc.teamcode.OpModes.testing;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.OpModes.auton.AutonBase;

@Config
//@Deprecated
@Autonomous(group="Tuning")
public class BackAndForthTest extends AutonBase {
    enum direction {
        FORWARD,
        BACKWARD
    }
    direction dir = direction.FORWARD;
    public static int numruns = 1;
    @Override
    public void runOpMode() {
        poscontroller = new PIDController(0.1, 0, 0);
        initialize();
        resetForStart();
        swerve.setAuton();
        slides.reset();
        waitForStart();
        int runs = 0;
        double piderror = 0;
        poscontroller.setPID(0.1, 0, 0);
        poscontroller.setSetPoint(0);
        tolerance = 0.5;
        swerve.headingLock(true, 0);
        while (opModeIsActive()) {
            update();
            if (runs >= numruns) {
                requestOpModeStop();
            } else {
                switch (dir) {
                    case FORWARD:
                        if (pidToPosition(10, 0)) {
                            dir = direction.BACKWARD;
                            runs++;
                            swerve.stop();
                            sleep(1000);
                            telemetry.addData("hi", "hi");
                            Pose2d odopose = odometry.getPose();
                            piderror += Math.sqrt(Math.pow(odopose.getX()-10, 2) + Math.pow(odopose.getY(), 2));
                        }
                        break;
                    case BACKWARD:
                        if (pidToPosition(0, 0)) {
                            dir = direction.FORWARD;
                            runs++;
                            swerve.stop();
                            sleep(1000);
                            Pose2d odopose = odometry.getPose();
                            piderror += Math.sqrt(Math.pow(odopose.getX(), 2) + Math.pow(odopose.getY(), 2));
                        }
                        break;
                }
            }
            if (runs != 0) {
                telemetry.addData("Average PID Position Error", piderror / runs);
            }
            telemetry.update();
        }
    }
}
