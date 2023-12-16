package com.team1701.lib.drivers.gyros;

import java.util.function.Supplier;

import com.team1701.robot.Constants;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class GyroIOSim implements GyroIO {
    private Supplier<ChassisSpeeds> mChassisSpeedsSupplier;

    public GyroIOSim(Supplier<ChassisSpeeds> chassisSpeedsSupplier) {
        mChassisSpeedsSupplier = chassisSpeedsSupplier;
    }

    public void updateInputs(GyroInputs inputs) {
        var chassisSpeeds = mChassisSpeedsSupplier.get();
        inputs.yawVelocityRadPerSec = chassisSpeeds.omegaRadiansPerSecond;
        inputs.yawPositionRad += chassisSpeeds.omegaRadiansPerSecond * Constants.kLoopPeriodSeconds;
    }
}
