package com.team1701.lib.drivers.motors;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class MotorIOSim implements MotorIO {
    private final DCMotorSim mSim;
    private final PIDController mController;
    private final double mReduction;
    private final double mLoopPeriodSeconds;
    private double mFeedForward;
    private double mVelocityRadiansPerSecond;
    private double mPositionRadians;

    public MotorIOSim(DCMotor motor, double reduction, double jKgMetersSquared, double loopPeriodSeconds) {
        mSim = new DCMotorSim(motor, 1.0 / reduction, jKgMetersSquared);
        mController = new PIDController(0.0, 0.0, 0.0, loopPeriodSeconds);
        mReduction = reduction;
        mLoopPeriodSeconds = loopPeriodSeconds;
    }

    @Override
    public void updateInputs(MotorInputs inputs) {
        if (DriverStation.isDisabled()) {
            mSim.setInputVoltage(0);
        }

        mSim.update(mLoopPeriodSeconds);

        mVelocityRadiansPerSecond = mSim.getAngularVelocityRadPerSec() / mReduction;
        mPositionRadians += mVelocityRadiansPerSecond * mLoopPeriodSeconds;
        inputs.positionRadians = mPositionRadians;
        inputs.velocityRadiansPerSecond = mVelocityRadiansPerSecond;
    }

    @Override
    public void setPositionControl(Rotation2d position) {
        mController.setSetpoint(position.getRadians());
        setVoltageOutput(mController.calculate(mPositionRadians) + position.getRadians() * mFeedForward);
    }

    @Override
    public void setVelocityControl(double velocityRadiansPerSecond) {
        mController.setSetpoint(velocityRadiansPerSecond);
        setVoltageOutput(mController.calculate(mVelocityRadiansPerSecond) + velocityRadiansPerSecond * mFeedForward);
    }

    @Override
    public void setPercentOutput(double percentage) {
        setVoltageOutput(percentage * 12.0);
    }

    @Override
    public void setVoltageOutput(double volts) {
        var appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
        mSim.setInputVoltage(appliedVolts);
    }

    @Override
    public void setBreakMode(boolean enable) {}

    @Override
    public void setPID(double ff, double p, double i, double d) {
        mFeedForward = ff;
        mController.setPID(p, i, d);
    }

    public void enableContinuousInput(double minimumInput, double maximumInput) {
        mController.enableContinuousInput(minimumInput, maximumInput);
    }

    public void disableContinuousInput() {
        mController.disableContinuousInput();
    }
}
