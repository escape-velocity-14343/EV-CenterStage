package org.firstinspires.ftc.teamcode.drivers;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ToggleTelemetry {
    Telemetry telemetry;
    public boolean enabled = true;
    public ToggleTelemetry(Telemetry telemetry) {
        this.telemetry = telemetry;
    }
    public void addData(String key, Object data) {
        if (enabled)
            telemetry.addData(key,data);
    }
    public void addLine(String line) {
        if (enabled)
            telemetry.addLine(line);
    }
    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
    }
}
