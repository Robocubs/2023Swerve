package com.team1701.robot.commands;

import com.team1701.lib.swerve.SwerveSetpointGenerator.KinematicLimits;
import com.team1701.robot.Constants;
import com.team1701.robot.estimation.PoseEstimator;
import com.team1701.robot.subsystems.drive.Drive;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;

public class DriveToPoseCommand extends Command {
    private final Drive mDrive;
    private final Pose2d mTargetPose;
    private final PIDController mPositionController;
    private final TrapezoidProfile mPositionProfile;
    private final PIDController mRotationController;
    private final TrapezoidProfile mRotationProfile;

    private Pose2d mSetpoint = new Pose2d();

    public DriveToPoseCommand(Drive drive, Pose2d pose, KinematicLimits kinematicLimits) {
        mDrive = drive;
        mTargetPose = pose;

        mPositionController = new PIDController(2, 0, 0, Constants.kLoopPeriodSeconds);
        mPositionController.setTolerance(0.01);
        mPositionProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(
                kinematicLimits.kMaxDriveVelocity, kinematicLimits.kMaxDriveAcceleration));

        var moduleRadius = Math.hypot(Constants.Drive.kTrackWidthMeters / 2.0, Constants.Drive.kWheelbaseMeters / 2.0);
        mRotationController = new PIDController(4, 0, 0, Constants.kLoopPeriodSeconds);
        mRotationController.setTolerance(Rotation2d.fromDegrees(0.5).getRadians());
        mRotationController.enableContinuousInput(-Math.PI, Math.PI);
        mRotationProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(
                kinematicLimits.kMaxDriveVelocity / moduleRadius,
                kinematicLimits.kMaxDriveAcceleration / moduleRadius));

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        mDrive.setKinematicLimits(Constants.Drive.kUncappedKinematicLimits);
        mSetpoint = PoseEstimator.getInstance().getPose2d();
        mPositionController.reset();
        mRotationController.reset();
    }

    @Override
    public void execute() {
        var currentPose = PoseEstimator.getInstance().getPose2d();
        var translationToTarget = mTargetPose.getTranslation().minus(currentPose.getTranslation());
        var fieldRelativeChassisSpeeds = mDrive.getFieldRelativeVelocity();
        var directionToTarget = translationToTarget.getAngle();
        var velocityToTarget = new Translation2d(
                        fieldRelativeChassisSpeeds.vxMetersPerSecond, fieldRelativeChassisSpeeds.vyMetersPerSecond)
                .rotateBy(directionToTarget)
                .getX();

        var positionCurrentState = new TrapezoidProfile.State(0, velocityToTarget);
        var positionTargetState = new TrapezoidProfile.State(translationToTarget.getNorm(), 0);
        var positionNewState =
                mPositionProfile.calculate(Constants.kLoopPeriodSeconds, positionCurrentState, positionTargetState);
        var feedForwardVelocity = new Translation2d(positionNewState.velocity, translationToTarget.getAngle());

        var positionError = mSetpoint.getTranslation().minus(currentPose.getTranslation());
        var positionPidOutput = positionError.times(mPositionController.calculate(0, positionError.getNorm()));

        var rotationCurrentState = new TrapezoidProfile.State(
                currentPose.getRotation().getRadians(),
                mTargetPose.getRotation().getRadians());
        var rotationTargetState =
                new TrapezoidProfile.State(mTargetPose.getRotation().getRadians(), 0);
        var rotationNewState =
                mRotationProfile.calculate(Constants.kLoopPeriodSeconds, rotationCurrentState, rotationTargetState);
        var feedForwardRotationalVelocity = rotationNewState.velocity;

        var rotationError = mSetpoint.getRotation().minus(currentPose.getRotation());
        var rotationPidOutput = mRotationController.calculate(rotationError.getRadians());

        mSetpoint = new Pose2d(
                currentPose.getTranslation().plus(new Translation2d(positionNewState.position, directionToTarget)),
                Rotation2d.fromRadians(rotationNewState.position));

        mDrive.setVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(
                feedForwardVelocity.getX() + positionPidOutput.getX(),
                feedForwardVelocity.getY() + positionPidOutput.getY(),
                feedForwardRotationalVelocity + rotationPidOutput,
                currentPose.getRotation()));
    }

    @Override
    public void end(boolean interrupted) {
        mDrive.stop();
    }

    public boolean atTargetPose() {
        return mPositionController.atSetpoint() && mRotationController.atSetpoint();
    }
}
