package com.team1701.lib.drivers.motors;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface MotorIO {
    @AutoLog
    public static class MotorInputs {
        public double positionRadians;
        public double velocityRadiansPerSecond;
    }

    public default void updateInputs(MotorInputs inputs) {}

    public default void setPositionControl(Rotation2d position) {}

    public default void setVelocityControl(double velocityRadiansPerSecond) {}

    public default void setPercentOutput(double percentage) {}

    public default void setVoltageOutput(double volts) {}

    public default void setBreakMode(boolean enable) {}

    public default void setPID(double ff, double p, double i, double d) {}
}
