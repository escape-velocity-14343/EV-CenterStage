package org.firstinspires.ftc.teamcode.timeutils;

import com.qualcomm.robotcore.util.ElapsedTime;

public class LoopUtil {
    public LoopUtil instance = new LoopUtil();

    private ElapsedTime timer = new ElapsedTime();
    private long lastNanos = 0;
    private boolean hasRun = false;
    private double lowPassGain = 0.5;
    private LoopTime lowPassOut = new LoopTime(0);
    private LoopTime currLoopTime;

    public LoopUtil getInstance() {
        return instance;
    }


    private class LoopTime {
        private long nanos;
        private double hz;

        public LoopTime(long nanos) {
            this.nanos = nanos;
            this.hz = 1e9 / ((double) nanos);
        }

        public LoopTime(double hz) {
            this.hz = hz;
            this.nanos = (long) (1e9/hz);
        }

        public long getNanos() {
            return this.nanos;
        }

        public double getHz() {
            return this.hz;
        }

    }

    public void update() {
        if (!hasRun) {
            hasRun = true;
            timer.reset();
        } else {
            // update curr lt
            long nanos = timer.nanoseconds();
            currLoopTime = new LoopTime(nanos-lastNanos);
            lastNanos = nanos;

            // update lowpass
            lowPassOut = new LoopTime(lowPassOut.getHz() * (1-lowPassGain) + currLoopTime.getHz() * lowPassGain);
        }
    }

    public double getLowPass() {
        return lowPassOut.getHz();
    }
}
