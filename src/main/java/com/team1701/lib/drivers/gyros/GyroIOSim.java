package com.team1701.lib.drivers.gyros;

import java.util.function.Supplier;

import com.team1701.lib.util.SignalSamplingThread;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;

public class GyroIOSim implements GyroIO {
    private Supplier<Rotation2d> mYawSupplier;
    private boolean mYawSamplingEnabled;

    public GyroIOSim(Supplier<ChassisSpeeds> chassisSpeedsSupplier, double loopPeriodSeconds) {
        this(new Supplier<Rotation2d>() {
            private double yawRadians;

            @Override
            public Rotation2d get() {
                yawRadians += chassisSpeedsSupplier.get().omegaRadiansPerSecond * loopPeriodSeconds;
                return Rotation2d.fromRadians(yawRadians);
            }
        });
    }

    public GyroIOSim(Supplier<Rotation2d> yawSupplier) {
        mYawSupplier = yawSupplier;
    }

    @Override
    public void updateInputs(GyroInputs inputs) {
        inputs.connected = true;
        inputs.yaw = mYawSupplier.get();
        if (mYawSamplingEnabled) {
            inputs.yawSamples = new Rotation2d[] {inputs.yaw};
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
