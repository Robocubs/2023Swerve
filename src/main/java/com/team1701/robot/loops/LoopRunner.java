package com.team1701.robot.loops;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.Logger;

public class LoopRunner implements Looper {
    private final List<Loop> mLoops;
    private final String mName;
    private double mTimestamp;
    private double mDT;
    private double mLoopTime;
    private boolean mRunning;

    public LoopRunner(String name) {
        mRunning = false;
        mLoops = new ArrayList<>();
        mName = name;
    }

    @Override
    public void register(Loop loop) {
        mLoops.add(loop);
    }

    public void start() {
        if (!mRunning) {
            System.out.println("Starting " + mName + " loops");
            mTimestamp = Timer.getFPGATimestamp();
            for (var loop : mLoops) {
                loop.onStart(mTimestamp);
            }
            mRunning = true;
        }
    }

    public void loop() {
        if (mRunning) {
            var realStart = Logger.getRealTimestamp();
            var start = Timer.getFPGATimestamp();
            for (var loop : mLoops) {
                loop.onLoop(start);
            }

            mLoopTime = (realStart - Logger.getRealTimestamp()) / 1000000.0;
            mDT = start - mTimestamp;
            mTimestamp = start;
        }
    }

    public void stop() {
        if (mRunning) {
            System.out.println("Stopping " + mName + " loops");
            mRunning = false;
            mTimestamp = Timer.getFPGATimestamp();
            for (var loop : mLoops) {
                System.out.println("Stopping " + loop);
                loop.onStop(mTimestamp);
            }
        }
    }

    public void outputTelemetry() {
        var prefix = "Looper/" + mName;
        Logger.recordOutput(prefix + "/Running", mRunning);
        Logger.recordOutput(prefix + "/TimeBetweenLoopsSec", mDT);
        Logger.recordOutput(prefix + "/LoopPeriodSec", mLoopTime);
    }
}
