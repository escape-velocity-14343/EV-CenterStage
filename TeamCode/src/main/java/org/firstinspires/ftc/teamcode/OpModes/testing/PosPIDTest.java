package org.firstinspires.ftc.teamcode.OpModes.testing;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.OpModes.auton.AutonBase;

@Config
//@Deprecated
@Autonomous(group="Tuning")
public class PosPIDTest extends AutonBase {

    public static double kPosP = 0.4;
    public static double kPosD = 0.1;
    public static double kPosI = 0;
    public static double PIDpos = 0;
    public static double pidtol = 0.5;
    public static double movePowerCap = 0.7;
    @Override
    public void runOpMode() {
        poscontroller = new PIDController(kPosP, kPosI, kPosD);
        initialize();
        resetForStart();
        swerve.setAuton();
        slides.reset();
        waitForStart();

        poscontroller.setSetPoint(0);
        tolerance = pidtol;
        swerve.headingLock(true, 0);
        while (opModeIsActive()) {
            poscontroller.setPID(kPosP, kPosI, kPosD);
            swerve.setLimits(movePowerCap, 1);

            update();
            pidToPosition(PIDpos, 0);
            telemetry.addData("Target", PIDpos);
            telemetry.addData("Position", odometry.getPose().getX());
            telemetry.update();
        }
    }
}
