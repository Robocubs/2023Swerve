package com.team1701.robot.subsystems.drive;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.util.Units;

public class GyroIOPigeon2 implements GyroIO {
    private final Pigeon2 pigeon;

    public GyroIOPigeon2(int pigeonID) {
        pigeon = new Pigeon2(pigeonID);
    }

    public void updateInputs(GyroInputs inputs) {
        inputs.rollPositionRad = Units.degreesToRadians(pigeon.getRoll().getValue());
        inputs.pitchPositionRad = Units.degreesToRadians(pigeon.getPitch().getValue());
        inputs.yawPositionRad = Units.degreesToRadians(pigeon.getYaw().getValue());
        inputs.rollVelocityRadPerSec =
                Units.degreesToRadians(pigeon.getAngularVelocityY().getValue());
        inputs.pitchVelocityRadPerSec =
                Units.degreesToRadians(pigeon.getAngularVelocityX().getValue());
        inputs.yawVelocityRadPerSec =
                Units.degreesToRadians(pigeon.getAngularVelocityZ().getValue());
    }
}
