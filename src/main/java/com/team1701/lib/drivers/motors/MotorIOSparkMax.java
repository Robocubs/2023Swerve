package com.team1701.lib.drivers.motors;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.team1701.lib.util.SparkMaxUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class MotorIOSparkMax implements MotorIO {
    private final CANSparkMax mMotor;
    private final RelativeEncoder mEncoder;
    private final SparkMaxPIDController mController;

    public MotorIOSparkMax(CANSparkMax motor) {
        mMotor = motor;
        mEncoder = motor.getEncoder();
        mController = motor.getPIDController();
    }

    @Override
    public void updateInputs(MotorInputs inputs) {
        inputs.positionRadians = Units.rotationsToRadians(SparkMaxUtil.cleanSparkMaxValue(0.0, mEncoder.getPosition()));
        inputs.velocityRadiansPerSecond =
                Units.rotationsToRadians(SparkMaxUtil.cleanSparkMaxValue(0.0, mEncoder.getVelocity()) / 60);
    }

    @Override
    public void setPositionControl(Rotation2d position) {
        mController.setReference(position.getRotations(), CANSparkMax.ControlType.kPosition);
    }

    @Override
    public void setVelocityControl(double velocityRadiansPerSecond) {
        mController.setReference(
                Units.radiansToRotations(velocityRadiansPerSecond) * 60, CANSparkMax.ControlType.kVelocity);
    }

    @Override
    public void setPercentOutput(double percentage) {
        mController.setReference(percentage, CANSparkMax.ControlType.kDutyCycle);
    }

    @Override
    public void setVoltageOutput(double volts) {
        mController.setReference(volts, CANSparkMax.ControlType.kVoltage);
    }

    @Override
    public void setBreakMode(boolean enable) {
        mMotor.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
    }

    @Override
    public void setPID(double ff, double p, double i, double d) {
        mController.setFF(ff);
        mController.setP(p);
        mController.setI(i);
        mController.setD(d);
    }
}
