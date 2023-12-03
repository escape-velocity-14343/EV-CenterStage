package org.firstinspires.ftc.teamcode.OpModes.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Red Audience")
public class RedAudience extends AutonBase {
    @Override
    public void runOpMode() throws InterruptedException {

        initAuton(RunPos.RED_AUDIENCE, ScoreAmount.PRELOADS);
        setAlliance(true);
        setSide(false);
        run();
    }
}
