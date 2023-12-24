package com.team1701.robot.subsystems.drive;

import com.team1701.lib.drivers.encoders.EncoderIO;
import com.team1701.lib.drivers.encoders.EncoderInputsAutoLogged;
import com.team1701.lib.drivers.motors.MotorIO;
import com.team1701.lib.drivers.motors.MotorInputsAutoLogged;
import com.team1701.lib.util.GeometryUtil;
import com.team1701.robot.Constants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.littletonrobotics.junction.Logger;

public class SwerveModule {
    private final int mIndex;
    private final MotorIO mDriveMotorIO;
    private final MotorIO mSteerMotorIO;
    private final EncoderIO mSteerEncoderIO;
    private final MotorInputsAutoLogged mDriveMotorInputs = new MotorInputsAutoLogged();
    private final MotorInputsAutoLogged mSteerMotorInputs = new MotorInputsAutoLogged();
    private final EncoderInputsAutoLogged mSteerEncoderInputs = new EncoderInputsAutoLogged();

    private Rotation2d mMeasuredAngle = GeometryUtil.kRotationIdentity;
    private Rotation2d mAngleOffset = GeometryUtil.kRotationIdentity;
    private boolean mAngleOffsetNotInitialized = true;

    public SwerveModule(int index, SwerveModuleIO moduleIO) {
        mIndex = index;
        mDriveMotorIO = moduleIO.driveMotorIO;
        mSteerMotorIO = moduleIO.steerMotorIO;
        mSteerEncoderIO = moduleIO.steerEncoderIO;

        mDriveMotorIO.setPID(
                Constants.Drive.kDriveKf.get(), Constants.Drive.kDriveKp.get(), 0, Constants.Drive.kDriveKd.get());
        mSteerMotorIO.setPID(0, Constants.Drive.kSteerKp.get(), 0, Constants.Drive.kSteerKd.get());
    }

    // Separated from periodic to support thread locking of odometry inputs
    public void updateInputs() {
        mDriveMotorIO.updateInputs(mDriveMotorInputs);
        mSteerMotorIO.updateInputs(mSteerMotorInputs);
        mSteerEncoderIO.updateInputs(mSteerEncoderInputs);
    }

    public void periodic() {
        Logger.processInputs("Drive/Module/" + mIndex + "/Drive", mDriveMotorInputs);
        Logger.processInputs("Drive/Module/" + mIndex + "/Steer", mSteerMotorInputs);
        Logger.processInputs("Drive/Module/" + mIndex + "/AbsoluteEncoder", mSteerEncoderInputs);

        // Zero the steering motor position on the first loop after receiving an absolute encoder position
        if (mAngleOffsetNotInitialized && !mSteerEncoderInputs.position.equals(GeometryUtil.kRotationIdentity)) {
            zeroSteeringMotor();
            mAngleOffsetNotInitialized = false;
        }

        mMeasuredAngle = toModuleAngle(Rotation2d.fromRadians(mSteerMotorInputs.positionRadians));

        var hashCode = hashCode();
        if (Constants.Drive.kDriveKf.hasChanged(hashCode)
                || Constants.Drive.kDriveKp.hasChanged(hashCode)
                || Constants.Drive.kDriveKd.hasChanged(hashCode)) {
            mDriveMotorIO.setPID(
                    Constants.Drive.kDriveKf.get(), Constants.Drive.kDriveKp.get(), 0, Constants.Drive.kDriveKd.get());
        }

        if (Constants.Drive.kSteerKp.hasChanged(hashCode) || Constants.Drive.kSteerKd.hasChanged(hashCode)) {
            mSteerMotorIO.setPID(0, Constants.Drive.kSteerKp.get(), 0, Constants.Drive.kSteerKd.get());
        }
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                mDriveMotorInputs.positionRadians * Constants.Drive.kWheelRadiusMeters, mMeasuredAngle);
    }

    public SwerveModulePosition[] getPositionSamples() {
        var states = new SwerveModulePosition
                [Math.min(
                        mDriveMotorInputs.positionRadiansSamples.length,
                        mSteerMotorInputs.positionRadiansSamples.length)];

        for (int i = 0; i < states.length; i++) {
            states[i] = new SwerveModulePosition(
                    mDriveMotorInputs.positionRadiansSamples[i] * Constants.Drive.kWheelRadiusMeters,
                    toModuleAngle(Rotation2d.fromRadians(mSteerMotorInputs.positionRadiansSamples[i])));
        }

        return states;
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
                mDriveMotorInputs.velocityRadiansPerSecond * Constants.Drive.kWheelRadiusMeters, mMeasuredAngle);
    }

    public void setState(SwerveModuleState state) {
        mDriveMotorIO.setVelocityControl(state.speedMetersPerSecond / Constants.Drive.kWheelRadiusMeters);
        mSteerMotorIO.setPositionControl(state.angle.minus(mAngleOffset));
    }

    public void setOrient(Rotation2d steerAngle) {
        mDriveMotorIO.setPercentOutput(0);
        mSteerMotorIO.setPositionControl(steerAngle.minus(mAngleOffset));
    }

    public void setDriveBrakeMode(boolean enable) {
        mDriveMotorIO.setBreakMode(enable);
    }

    public void setSteerBrakeMode(boolean enable) {
        mSteerMotorIO.setBreakMode(enable);
    }

    public void zeroSteeringMotor() {
        mAngleOffset = mSteerEncoderInputs.position.minus(Rotation2d.fromRadians(mSteerMotorInputs.positionRadians));
        mMeasuredAngle = mSteerEncoderInputs.position;
    }

    public void stop() {
        mDriveMotorIO.setPercentOutput(0.0);
        mSteerMotorIO.setPercentOutput(0.0);
    }

    private Rotation2d toModuleAngle(Rotation2d steerMotorPosition) {
        return GeometryUtil.angleModulus(steerMotorPosition.plus(mAngleOffset));
    }
}
