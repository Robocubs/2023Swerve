package com.team1701.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface SwerveModuleIO {
    @AutoLog
    public static class SwerveModuleInputs {
        public double drivePositionRad;
        public double driveVelocityRadPerSec;
        public double steerAbsolutePositionRad;
        public double steerPositionRad;
        public double steerVelocityRadPerSec;
    }

    public default void updateInputs(SwerveModuleInputs inputs) {}

    public default void setWithVelocity(double driveVelocityRadPerSec, Rotation2d steerAngle) {}

    public default void setWithPercentOutput(double drivePercentage, Rotation2d steerAngle) {}
}
