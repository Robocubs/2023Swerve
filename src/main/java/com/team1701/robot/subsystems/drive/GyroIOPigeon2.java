package com.team1701.robot.subsystems.drive;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.sensors.Pigeon2;
import edu.wpi.first.math.util.Units;

public class GyroIOPigeon2 implements GyroIO {
    private final Pigeon2 pigeon;
    private final double[] yprDegrees = new double[3];
    private final double[] xyzDegreesPerSecond = new double[3];

    public GyroIOPigeon2(int pigeonID) {
        pigeon = new Pigeon2(pigeonID);
    }

    public void updateInputs(GyroInputs inputs) {
        pigeon.getYawPitchRoll(yprDegrees);
        pigeon.getRawGyro(xyzDegreesPerSecond);
        inputs.connected = pigeon.getLastError().equals(ErrorCode.OK);
        inputs.rollPositionRad = Units.degreesToRadians(yprDegrees[1]);
        inputs.pitchPositionRad = Units.degreesToRadians(-yprDegrees[2]);
        inputs.yawPositionRad = Units.degreesToRadians(yprDegrees[0]);
        inputs.rollVelocityRadPerSec = Units.degreesToRadians(xyzDegreesPerSecond[1]);
        inputs.pitchVelocityRadPerSec = Units.degreesToRadians(-xyzDegreesPerSecond[0]);
        inputs.yawVelocityRadPerSec = Units.degreesToRadians(xyzDegreesPerSecond[2]);
    }
}
