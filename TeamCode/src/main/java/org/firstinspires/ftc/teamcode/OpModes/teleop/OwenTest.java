package org.firstinspires.ftc.teamcode.OpModes.teleop;

import org.firstinspires.ftc.teamcode.drivers.PhoneNumber;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

public class OwenTest extends Robot {

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        PhoneNumber owen = new PhoneNumber(9549131665L, "Owen Friendman");
        owen.call();
    }
}
