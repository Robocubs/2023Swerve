package com.team1701.robot.subsystems.drive;

public class SwerveModuleConfiguration {
    public final int driveId;
    public final int steerId;
    public final int steerEncoderId;

    public SwerveModuleConfiguration(int driveId, int steerId, int steerEncoderId) {
        this.driveId = driveId;
        this.steerId = steerId;
        this.steerEncoderId = steerEncoderId;
    }
}
