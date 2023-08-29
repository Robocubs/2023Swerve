package com.team1701.robot.subsystems.drive;

import com.team1701.lib.util.GeometryUtil;
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
        var optimizedState = SwerveModuleState.optimize(state, mMeasuredAngle);
        mDesiredVelocityRadPerSec = optimizedState.speedMetersPerSecond / Constants.Drive.kWheelRadiusMeters;
        mDesiredAngle = optimizedState.angle;
        mOrienting = false;
    }

    public void setOrient(Rotation2d steerAngle) {
        mDesiredVelocityRadPerSec = 0;
        mDesiredAngle = steerAngle;
        mOrienting = true;
    }

    @Override
    public void readPeriodicInputs() {
        mIO.updateInputs(mInputs);
        Logger.getInstance().processInputs("Drive/Module/" + mIndex, mInputs);
        mMeasuredAngle =
                new Rotation2d(MathUtil.angleModulus(mInputs.steerPositionRad * Constants.Drive.kSteerReduction));
    }

    @Override
    public void writePeriodicOutputs() {
        if (mOrienting) {
            mIO.setWithPercentOutput(0, mDesiredAngle);
        } else {
            mIO.setWithVelocity(mDesiredVelocityRadPerSec, mDesiredAngle);
        }
    }

    @Override
    public void stop() {}

    @Override
    public void outputTelemetry() {}
}
