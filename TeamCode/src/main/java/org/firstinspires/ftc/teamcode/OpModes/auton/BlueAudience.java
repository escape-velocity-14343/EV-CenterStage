package org.firstinspires.ftc.teamcode.OpModes.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Blue Audience")
public class BlueAudience extends AutonBase {
    @Override
    public void runOpMode() throws InterruptedException {

        initAuton(RunPos.BLUE_AUDIENCE, ScoreAmount.PRELOADS);
        setAlliance(false);
        setSide(false);
        run();
    }
}
