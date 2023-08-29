package com.team1701.robot.loops;

public interface Loop {
    void onStart(double timestamp);

    void onLoop(double timestamp);

    void onStop(double timestamp);

    String getDisplayName();
}
