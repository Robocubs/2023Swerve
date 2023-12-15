package com.team1701.robot.subsystems.drive;

import com.team1701.lib.util.GeometryUtil;
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
    private final SwerveModuleIO mIO;
    private final SwerveModuleInputsAutoLogged mInputs = new SwerveModuleInputsAutoLogged();

    private double mDesiredVelocityRadPerSec;
    private Rotation2d mDesiredAngle = GeometryUtil.kRotationIdentity;
    private Rotation2d mMeasuredAngle = GeometryUtil.kRotationIdentity;
    private double mAngleOffsetRadians;
    private boolean mOrienting;

    public SwerveModule(int index, SwerveModuleIO io) {
        mIndex = index;
        mIO = io;
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                mInputs.drivePositionRad * Constants.Drive.kDriveReduction * Constants.Drive.kWheelRadiusMeters,
                mMeasuredAngle);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
                mInputs.driveVelocityRadPerSec * Constants.Drive.kDriveReduction * Constants.Drive.kWheelRadiusMeters,
                mMeasuredAngle);
    }

    public void setState(SwerveModuleState state) {
        // var optimizedState = SwerveModuleState.optimize(state, mMeasuredAngle);
        var optimizedState = state;
        mDesiredVelocityRadPerSec = optimizedState.speedMetersPerSecond / Constants.Drive.kWheelRadiusMeters;
        mDesiredAngle = optimizedState.angle;
        mOrienting = false;
    }

    public void setOrient(Rotation2d steerAngle) {
        mDesiredVelocityRadPerSec = 0;
        mDesiredAngle = steerAngle;
        mOrienting = true;
    }

    public void setDriveBrakeMode(boolean enable) {
        mIO.setDriveBrakeMode(enable);
    }

    public void setSteerBrakeMode(boolean enable) {
        mIO.setSteerBrakeMode(enable);
    }

    public void zeroSteeringMotor() {
        mAngleOffsetRadians = MathUtil.angleModulus(
                mInputs.steerAbsolutePositionRad - mInputs.steerPositionRad * Constants.Drive.kSteerReduction);
        mMeasuredAngle = new Rotation2d(MathUtil.angleModulus(
                mInputs.steerPositionRad * Constants.Drive.kSteerReduction + mAngleOffsetRadians));
    }

    @Override
    public void readPeriodicInputs() {
        mIO.updateInputs(mInputs);
        Logger.getInstance().processInputs("Drive/Module/" + mIndex, mInputs);
        mMeasuredAngle =
                new Rotation2d(MathUtil.angleModulus(mInputs.steerPositionRad * Constants.Drive.kSteerReduction)
                        + mAngleOffsetRadians);
        Logger.getInstance().recordOutput("Drive/Module/MeasuredAngle", mAngleOffsetRadians);
    }

    @Override
    public void writePeriodicOutputs() {
        if (mOrienting
                || Util.epsilonEquals(mDesiredVelocityRadPerSec, 0)
                        && Util.epsilonEquals(mInputs.driveVelocityRadPerSec, 0.0, 0.2)) {
            mIO.setWithPercentOutput(0, mDesiredAngle.minus(Rotation2d.fromRadians(mAngleOffsetRadians)));
        } else {
            mIO.setWithVelocity(
                    mDesiredVelocityRadPerSec, mDesiredAngle.minus(Rotation2d.fromRadians(mAngleOffsetRadians)));
        }
    }

    @Override
    public void stop() {}

    @Override
    public void outputTelemetry() {}
}
