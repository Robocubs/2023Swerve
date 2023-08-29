package com.team1701.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {
    @AutoLog
    public static class GyroInputs {
        public boolean connected;
        public double rollPositionRad;
        public double pitchPositionRad;
        public double yawPositionRad;
        public double rollVelocityRadPerSec;
        public double pitchVelocityRadPerSec;
        public double yawVelocityRadPerSec;
    }

    public default void updateInputs(GyroInputs inputs) {}
}
