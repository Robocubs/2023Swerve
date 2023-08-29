package com.team1701.robot.subsystems.drive;

import java.util.function.Supplier;

import com.team1701.robot.Constants;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class GyroIOSim implements GyroIO {
    private Supplier<ChassisSpeeds> mGetChassisSpeeds;

    public GyroIOSim(Supplier<ChassisSpeeds> getChassisSpeeds) {
        mGetChassisSpeeds = getChassisSpeeds;
    }

    public void updateInputs(GyroInputs inputs) {
        var chassisSpeeds = mGetChassisSpeeds.get();
        inputs.yawVelocityRadPerSec = chassisSpeeds.omegaRadiansPerSecond;
        inputs.yawPositionRad += chassisSpeeds.omegaRadiansPerSecond * Constants.kLoopPeriodSeconds;
    }
}
