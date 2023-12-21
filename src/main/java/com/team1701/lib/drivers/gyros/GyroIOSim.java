package com.team1701.lib.drivers.gyros;

import java.util.function.Supplier;

import com.team1701.lib.util.SignalSamplingThread;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;

public class GyroIOSim implements GyroIO {
    private final Supplier<ChassisSpeeds> mChassisSpeedsSupplier;
    private final double mLoopPeriodSeconds;

    private boolean mYawSamplingEnabled;

    public GyroIOSim(Supplier<ChassisSpeeds> chassisSpeedsSupplier, double loopPeriodSeconds) {
        mChassisSpeedsSupplier = chassisSpeedsSupplier;
        mLoopPeriodSeconds = loopPeriodSeconds;
    }

    public void updateInputs(GyroInputs inputs) {
        inputs.connected = true;

        var chassisSpeeds = mChassisSpeedsSupplier.get();
        var yaw = Rotation2d.fromRadians(
                inputs.yaw.getRadians() + chassisSpeeds.omegaRadiansPerSecond * mLoopPeriodSeconds);
        inputs.yaw = yaw;

        if (mYawSamplingEnabled) {
            inputs.yawSamples = new Rotation2d[] {yaw};
        }
    }

    @Override
    public void enableYawSampling(SignalSamplingThread samplingThread) {
        if (mYawSamplingEnabled) {
            DriverStation.reportWarning("Yaw sampling already enabled", false);
            return;
        }

        mYawSamplingEnabled = true;
    }
}
