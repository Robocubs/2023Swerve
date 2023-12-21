package com.team1701.robot.subsystems.drive;

import com.team1701.lib.drivers.encoders.EncoderIO;
import com.team1701.lib.drivers.encoders.EncoderInputsAutoLogged;
import com.team1701.lib.drivers.motors.MotorIO;
import com.team1701.lib.drivers.motors.MotorInputsAutoLogged;
import com.team1701.lib.util.GeometryUtil;
import com.team1701.lib.util.SignalSamplingThread;
import com.team1701.lib.util.Util;
import com.team1701.robot.Constants;
import com.team1701.robot.subsystems.Subsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.littletonrobotics.junction.Logger;

public class SwerveModule extends Subsystem {
    private final int mIndex;
    private final MotorIO mDriveMotorIO;
    private final MotorIO mSteerMotorIO;
    private final EncoderIO mSteerEncoderIO;
    private final MotorInputsAutoLogged mDriveMotorInputs = new MotorInputsAutoLogged();
    private final MotorInputsAutoLogged mSteerMotorInputs = new MotorInputsAutoLogged();
    private final EncoderInputsAutoLogged mSteerEncoderInputs = new EncoderInputsAutoLogged();

    private double mDesiredVelocityRadiansPerSecond;
    private Rotation2d mDesiredAngle = GeometryUtil.kRotationIdentity;
    private Rotation2d mMeasuredAngle = GeometryUtil.kRotationIdentity;
    private double mAngleOffsetRadians;
    private boolean mOrienting;

    public SwerveModule(
            int index,
            MotorIO driveMotorIO,
            MotorIO steerMotorIO,
            EncoderIO steerEncoderIO,
            SignalSamplingThread odometryThread) {
        mIndex = index;
        mDriveMotorIO = driveMotorIO;
        mSteerMotorIO = steerMotorIO;
        mSteerEncoderIO = steerEncoderIO;

        mDriveMotorIO.setPID(
                Constants.Drive.kDriveKf.get(), Constants.Drive.kDriveKp.get(), 0, Constants.Drive.kDriveKd.get());
        mSteerMotorIO.setPID(0, Constants.Drive.kSteerKp.get(), 0, Constants.Drive.kSteerKd.get());

        mDriveMotorIO.enablePositionSampling(odometryThread);
        mSteerMotorIO.enablePositionSampling(odometryThread);
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                mDriveMotorInputs.positionRadians
                        * Constants.Drive.kDriveReduction
                        * Constants.Drive.kWheelRadiusMeters,
                mMeasuredAngle);
    }

    public SwerveModulePosition[] getPositionSamples() {
        var states = new SwerveModulePosition
                [Math.min(
                        mDriveMotorInputs.positionRadiansSamples.length,
                        mSteerMotorInputs.positionRadiansSamples.length)];

        for (int i = 0; i < states.length; i++) {
            states[i] = new SwerveModulePosition(
                    mDriveMotorInputs.positionRadiansSamples[i]
                            * Constants.Drive.kDriveReduction
                            * Constants.Drive.kWheelRadiusMeters,
                    new Rotation2d(MathUtil.angleModulus(
                            mSteerMotorInputs.positionRadiansSamples[i] * Constants.Drive.kSteerReduction
                                    + mAngleOffsetRadians)));
        }

        return states;
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
                mDriveMotorInputs.velocityRadiansPerSecond
                        * Constants.Drive.kDriveReduction
                        * Constants.Drive.kWheelRadiusMeters,
                mMeasuredAngle);
    }

    public void setState(SwerveModuleState state) {
        mDesiredVelocityRadiansPerSecond =
                state.speedMetersPerSecond / Constants.Drive.kDriveReduction / Constants.Drive.kWheelRadiusMeters;
        mDesiredAngle = state.angle;
        mOrienting = false;
    }

    public void setOrient(Rotation2d steerAngle) {
        mDesiredVelocityRadiansPerSecond = 0;
        mDesiredAngle = steerAngle;
        mOrienting = true;
    }

    public void setDriveBrakeMode(boolean enable) {
        mDriveMotorIO.setBreakMode(enable);
    }

    public void setSteerBrakeMode(boolean enable) {
        mSteerMotorIO.setBreakMode(enable);
    }

    public void zeroSteeringMotor() {
        mAngleOffsetRadians = MathUtil.angleModulus(mSteerEncoderInputs.position.getRadians()
                - mSteerMotorInputs.positionRadians * Constants.Drive.kSteerReduction);
        mMeasuredAngle = new Rotation2d(MathUtil.angleModulus(
                mSteerMotorInputs.positionRadians * Constants.Drive.kSteerReduction + mAngleOffsetRadians));
    }

    @Override
    public void readPeriodicInputs() {
        mDriveMotorIO.updateInputs(mDriveMotorInputs);
        mSteerMotorIO.updateInputs(mSteerMotorInputs);
        mSteerEncoderIO.updateInputs(mSteerEncoderInputs);
        Logger.processInputs("Drive/Module/" + mIndex + "/Drive", mDriveMotorInputs);
        Logger.processInputs("Drive/Module/" + mIndex + "/Steer", mSteerMotorInputs);
        Logger.processInputs("Drive/Module/" + mIndex + "/AbsoluteEncoder", mSteerEncoderInputs);
        mMeasuredAngle = new Rotation2d(MathUtil.angleModulus(
                mSteerMotorInputs.positionRadians * Constants.Drive.kSteerReduction + mAngleOffsetRadians));
    }

    @Override
    public void writePeriodicOutputs() {
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

        var velocityNearZero = Util.epsilonEquals(mDesiredVelocityRadiansPerSecond, 0)
                && Util.epsilonEquals(mDriveMotorInputs.velocityRadiansPerSecond, 0, 0.2);
        if (mOrienting || velocityNearZero) {
            mDriveMotorIO.setPercentOutput(0);
        } else {
            mDriveMotorIO.setVelocityControl(mDesiredVelocityRadiansPerSecond);
        }

        mSteerMotorIO.setPositionControl(
                mDesiredAngle.minus(Rotation2d.fromRadians(mAngleOffsetRadians)).div(Constants.Drive.kSteerReduction));
    }

    @Override
    public void stop() {}

    @Override
    public void outputTelemetry() {}
}
