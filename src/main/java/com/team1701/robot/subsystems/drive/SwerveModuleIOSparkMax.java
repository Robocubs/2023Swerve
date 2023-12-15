package com.team1701.robot.subsystems.drive;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.team1701.lib.util.SparkMaxUtil;
import com.team1701.robot.Constants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogEncoder;
import org.littletonrobotics.junction.Logger;

public class SwerveModuleIOSparkMax implements SwerveModuleIO {
    private final CANSparkMax mDriveMotor;
    private final RelativeEncoder mDriveEncoder;
    private final SparkMaxPIDController mDriveController;
    private final CANSparkMax mSteerMotor;
    private final RelativeEncoder mSteerEncoder;
    private final SparkMaxPIDController mSteerController;
    private final AnalogEncoder mSteerAbsoluteEncoder;
    private final double mSteerReduction;
    private final int mSteerId;

    public SwerveModuleIOSparkMax(
            int driveId,
            int steerId,
            int steerEncoderId,
            boolean driveInverted,
            boolean steerInverted,
            double steerReduction,
            Rotation2d steerEncoderOffset) {
        mSteerId = steerId;
        mDriveMotor = new CANSparkMax(driveId, MotorType.kBrushless);
        mSteerMotor = new CANSparkMax(steerId, MotorType.kBrushless);
        mDriveEncoder = mDriveMotor.getEncoder();
        mSteerEncoder = mSteerMotor.getEncoder();
        mDriveController = mDriveMotor.getPIDController();
        mSteerController = mSteerMotor.getPIDController();
        mSteerAbsoluteEncoder = new AnalogEncoder(steerEncoderId);
        mSteerReduction = steerReduction;

        mSteerAbsoluteEncoder.setPositionOffset(MathUtil.inputModulus(steerEncoderOffset.getRotations(), 0, 1));

        // TODO: decide when to burn values
        // mDriveMotor.restoreFactoryDefaults();
        // mSteerMotor.restoreFactoryDefaults();

        mDriveMotor.setCANTimeout(500);
        mSteerMotor.setCANTimeout(500);

        for (var i = 0; i < 4; i++) {
            mDriveMotor.setInverted(driveInverted);
            mDriveMotor.setSmartCurrentLimit(40);
            mDriveMotor.enableVoltageCompensation(12.0);

            mDriveEncoder.setPosition(0.0);
            mDriveEncoder.setMeasurementPeriod(10);
            mDriveEncoder.setAverageDepth(2);

            mDriveController.setP(Constants.Drive.kDriveKp.get());
            mDriveController.setD(Constants.Drive.kDriveKd.get());
            mDriveController.setFF(Constants.Drive.kDriveKf.get());

            mSteerMotor.setInverted(steerInverted);
            mDriveMotor.setSmartCurrentLimit(30);
            mDriveMotor.enableVoltageCompensation(12.0);

            mSteerEncoder.setPosition(0.0);
            mSteerEncoder.setMeasurementPeriod(10);
            mSteerEncoder.setAverageDepth(2);

            mSteerController.setP(Constants.Drive.kSteerKp.get());
            mSteerController.setD(Constants.Drive.kSteerKd.get());

            mSteerController.setPositionPIDWrappingEnabled(true);
            mSteerController.setPositionPIDWrappingMinInput(0.0);
            mSteerController.setPositionPIDWrappingMaxInput(1.0 / steerReduction);
        }

        mDriveMotor.setCANTimeout(0);
        mSteerMotor.setCANTimeout(0);

        // mDriveMotor.burnFlash();
        // mSteerMotor.burnFlash();
    }

    public void updateInputs(SwerveModuleInputs inputs) {
        inputs.drivePositionRad =
                Units.rotationsToRadians(SparkMaxUtil.cleanSparkMaxValue(0.0, mDriveEncoder.getPosition()));
        inputs.driveVelocityRadPerSec =
                Units.rotationsToRadians(SparkMaxUtil.cleanSparkMaxValue(0.0, mDriveEncoder.getVelocity()) / 60);
        inputs.steerPositionRad =
                Units.rotationsToRadians(SparkMaxUtil.cleanSparkMaxValue(0.0, mSteerEncoder.getPosition()));
        inputs.steerVelocityRadPerSec =
                Units.rotationsToRadians(SparkMaxUtil.cleanSparkMaxValue(0.0, mSteerEncoder.getVelocity()) / 60);

        var absoluteEncoderRotations = mSteerAbsoluteEncoder.get();
        inputs.steerAbsolutePositionRad = Units.rotationsToRadians(
                absoluteEncoderRotations == Double.NaN || absoluteEncoderRotations == Double.POSITIVE_INFINITY
                        ? 0.0
                        : absoluteEncoderRotations);

        Logger.getInstance()
                .recordOutput("SwerveModule/" + mSteerId + "/RawSteerMeasured", mSteerEncoder.getPosition());
    }

    public void setWithVelocity(double driveVelocityRadPerSec, Rotation2d steerAngle) {
        var hashCode = hashCode();
        if (Constants.Drive.kDriveKf.hasChanged(hashCode)) {
            mDriveController.setFF(Constants.Drive.kDriveKf.get());
        }
        if (Constants.Drive.kDriveKp.hasChanged(hashCode)) {
            mDriveController.setP(Constants.Drive.kDriveKp.get());
        }
        if (Constants.Drive.kDriveKd.hasChanged(hashCode)) {
            mDriveController.setD(Constants.Drive.kDriveKd.get());
        }

        mDriveController.setReference(
                Units.radiansToRotations(driveVelocityRadPerSec) * 60, CANSparkMax.ControlType.kVelocity);
        setSteerAngle(steerAngle);
    }

    public void setWithPercentOutput(double drivePercentage, Rotation2d steerAngle) {
        mDriveController.setReference(drivePercentage, CANSparkMax.ControlType.kDutyCycle);
        setSteerAngle(steerAngle);
    }

    private void setSteerAngle(Rotation2d angle) {
        var hashCode = hashCode();
        if (Constants.Drive.kSteerKp.hasChanged(hashCode)) {
            mSteerController.setP(Constants.Drive.kSteerKp.get());
        }
        if (Constants.Drive.kSteerKd.hasChanged(hashCode)) {
            mSteerController.setD(Constants.Drive.kSteerKd.get());
        }

        mSteerController.setReference(
                MathUtil.inputModulus(angle.getRotations(), 0.0, 1.0) / mSteerReduction,
                CANSparkMax.ControlType.kPosition);
        Logger.getInstance()
                .recordOutput("SwerveModule/" + mSteerId + "/RawSteerRequest", angle.getRotations() / mSteerReduction);
    }

    public void setDriveBrakeMode(boolean enable) {
        mDriveMotor.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
    }

    public void setSteerBrakeMode(boolean enable) {
        mSteerMotor.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
    }
}
