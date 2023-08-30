package com.team1701.robot.subsystems.drive;

import com.team1701.robot.Constants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class SwerveModuleIOSim implements SwerveModuleIO {
    private final FlywheelSim mDriveSim =
            new FlywheelSim(DCMotor.getNEO(1), 1 / Constants.Drive.kDriveReduction, 0.025);
    private final FlywheelSim mSteerSim =
            new FlywheelSim(DCMotor.getNEO(1), 1 / Constants.Drive.kSteerReduction, 0.004);
    private final PIDController mDrivePID = new PIDController(0.0, 0.0, 0.0, Constants.kLoopPeriodSeconds);
    private final PIDController mSteerPID = new PIDController(0.0, 0.0, 0.0, Constants.kLoopPeriodSeconds);

    private double mDriveVelocityRadPerSec = 0.0;
    private double mSteerRelativePositionRad = 0.0;
    private double mSteerAbsolutePositionRad = Math.random() * 2.0 * Math.PI;

    public SwerveModuleIOSim() {
        System.out.println("[Init] Creating ModuleIOSim");
        mSteerPID.enableContinuousInput(-Math.PI, Math.PI);
    }

    public void updateInputs(SwerveModuleInputs inputs) {
        if (DriverStation.isDisabled()) {
            mDriveSim.setInputVoltage(0);
            mSteerSim.setInputVoltage(0);
        }

        mDriveSim.update(Constants.kLoopPeriodSeconds);
        mSteerSim.update(Constants.kLoopPeriodSeconds);

        var angleDiffRad = mSteerSim.getAngularVelocityRadPerSec() * Constants.kLoopPeriodSeconds;
        mSteerRelativePositionRad += angleDiffRad;
        mSteerAbsolutePositionRad += angleDiffRad;
        mSteerAbsolutePositionRad = MathUtil.inputModulus(mSteerAbsolutePositionRad, 0, 2 * Math.PI);

        mDriveVelocityRadPerSec = mDriveSim.getAngularVelocityRadPerSec();
        inputs.drivePositionRad +=
                mDriveVelocityRadPerSec / Constants.Drive.kDriveReduction * Constants.kLoopPeriodSeconds;
        inputs.driveVelocityRadPerSec = mDriveVelocityRadPerSec / Constants.Drive.kDriveReduction;

        inputs.steerAbsolutePositionRad = mSteerAbsolutePositionRad;
        inputs.steerPositionRad = mSteerRelativePositionRad / Constants.Drive.kSteerReduction;
        inputs.steerVelocityRadPerSec = mSteerSim.getAngularVelocityRadPerSec() / Constants.Drive.kSteerReduction;
    }

    public void setWithVelocity(double driveVelocityRadPerSec, Rotation2d steerAngle) {
        if (Constants.Drive.kDriveKp.hasChanged(hashCode()) || Constants.Drive.kDriveKd.hasChanged(hashCode())) {
            mDrivePID.setPID(Constants.Drive.kDriveKp.get(), 0, Constants.Drive.kDriveKd.get());
        }

        mDrivePID.setSetpoint(driveVelocityRadPerSec);
        setDriveVoltage(
                mDrivePID.calculate(mDriveVelocityRadPerSec) + driveVelocityRadPerSec * Constants.Drive.kDriveKf.get());
        setSteerAngle(steerAngle);
    }

    public void setWithPercentOutput(double drivePercentage, Rotation2d steerAngle) {
        setDriveVoltage(drivePercentage * 12.0);
        setSteerAngle(steerAngle);
    }

    private void setDriveVoltage(double volts) {
        var appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
        mDriveSim.setInputVoltage(appliedVolts);
    }

    private void setSteerAngle(Rotation2d angle) {
        if (Constants.Drive.kSteerKp.hasChanged(hashCode()) || Constants.Drive.kSteerKd.hasChanged(hashCode())) {
            mSteerPID.setPID(Constants.Drive.kSteerKp.get(), 0, Constants.Drive.kSteerKd.get());
        }

        mSteerPID.setSetpoint(angle.getRadians());
        var volts = mSteerPID.calculate(mSteerAbsolutePositionRad);
        var appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
        mSteerSim.setInputVoltage(appliedVolts);
    }
}
