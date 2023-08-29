package com.team1701.robot.subsystems;

import com.team1701.robot.loops.Looper;

public abstract class Subsystem {
    public void readPeriodicInputs() {}

    public void writePeriodicOutputs() {}

    public void registerEnabledLoops(Looper looper) {}

    public abstract void stop();

    public abstract void outputTelemetry();
}
