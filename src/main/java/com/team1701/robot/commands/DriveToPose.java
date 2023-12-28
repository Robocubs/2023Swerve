package com.team1701.robot.commands;

import com.team1701.lib.swerve.SwerveSetpointGenerator.KinematicLimits;
import com.team1701.lib.util.GeometryUtil;
import com.team1701.lib.util.LoggedTunableNumber;
import com.team1701.lib.util.Util;
import com.team1701.robot.Constants;
import com.team1701.robot.estimation.PoseEstimator;
import com.team1701.robot.subsystems.drive.Drive;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.Logger;

public class DriveToPose extends Command {
    private static final String kLoggingPrefix = "Command/DriveToPose/";
    private static final double kModuleRadius = Constants.Drive.kModuleRadius;
    private static final KinematicLimits kMaxKinematicLimits = Constants.Drive.kFastTrapezoidalKinematicLimits;
    private static final LoggedTunableNumber kMaxVelocity =
            new LoggedTunableNumber(kLoggingPrefix + "MaxVelocity", kMaxKinematicLimits.kMaxDriveVelocity);
    private static final LoggedTunableNumber kMaxAcceleration =
            new LoggedTunableNumber(kLoggingPrefix + "MaxAcceleration", kMaxKinematicLimits.kMaxDriveVelocity / 2.0);
    private static final LoggedTunableNumber kMaxAngularVelocity = new LoggedTunableNumber(
            kLoggingPrefix + "MaxAngularVelocity", kMaxKinematicLimits.kMaxDriveVelocity / kModuleRadius);
    private static final LoggedTunableNumber kMaxAngularAcceleration =
            new LoggedTunableNumber(kLoggingPrefix + "MaxAngularAcceleration", kMaxAngularVelocity.get() / 2.0);
    private static final LoggedTunableNumber kPositionKp = new LoggedTunableNumber(kLoggingPrefix + "PositionKp", 6.0);
    private static final LoggedTunableNumber kPositionKi = new LoggedTunableNumber(kLoggingPrefix + "PositionKi", 0.0);
    private static final LoggedTunableNumber kPositionKd = new LoggedTunableNumber(kLoggingPrefix + "PositionKd", 0.0);
    private static final LoggedTunableNumber kRotationKp = new LoggedTunableNumber(kLoggingPrefix + "RotationKp", 4.0);
    private static final LoggedTunableNumber kRotationKi = new LoggedTunableNumber(kLoggingPrefix + "RotationKi", 0.0);
    private static final LoggedTunableNumber kRotationKd = new LoggedTunableNumber(kLoggingPrefix + "RotationKd", 0.0);
    private static final LoggedTunableNumber kPositionToleranceMeters =
            new LoggedTunableNumber(kLoggingPrefix + "PositionToleranceMeters", 0.01);
    private static final LoggedTunableNumber kRotationToleranceRadians =
            new LoggedTunableNumber(kLoggingPrefix + "RotationToleranceRadians", 0.01);

    private final Drive mDrive;
    private final Pose2d mTargetPose;
    private final KinematicLimits mKinematicLimits;
    private final PIDController mPositionController;
    private final PIDController mRotationController;

    private Pose2d mSetpoint = GeometryUtil.kPoseIdentity;
    private TrapezoidProfile mPositionProfile;
    private TrapezoidProfile mRotationProfile;
    private TrapezoidProfile.State mPositionState = new TrapezoidProfile.State();
    private TrapezoidProfile.State mRotationState = new TrapezoidProfile.State();

    DriveToPose(Drive drive, Pose2d pose, KinematicLimits kinematicLimits) {
        mDrive = drive;
        mTargetPose = pose;
        mKinematicLimits = kinematicLimits;

        mPositionController = new PIDController(0.0, 0.0, 0.0, Constants.kLoopPeriodSeconds);
        mPositionProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(0.0, 0.0));

        mRotationController = new PIDController(0.0, 0.0, 0.0, Constants.kLoopPeriodSeconds);
        mRotationController.enableContinuousInput(-Math.PI, Math.PI);
        mRotationProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(0.0, 0.0));

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        mDrive.setKinematicLimits(Constants.Drive.kFastKinematicLimits);
        mSetpoint = PoseEstimator.getInstance().getPose2d();

        mPositionController.reset();
        mRotationController.reset();

        var currentPose = PoseEstimator.getInstance().getPose2d();
        var translationToTarget = mTargetPose.getTranslation().minus(currentPose.getTranslation());
        var rotationToTarget = translationToTarget.getAngle();
        var fieldRelativeChassisSpeeds = mDrive.getFieldRelativeVelocity();
        var velocityToTarget =
                ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeChassisSpeeds, rotationToTarget).vxMetersPerSecond;
        mPositionState = new TrapezoidProfile.State(translationToTarget.getNorm(), -velocityToTarget);
        mRotationState = new TrapezoidProfile.State(
                MathUtil.inputModulus(
                        currentPose.getRotation().getRadians(),
                        mTargetPose.getRotation().getRadians() - Math.PI,
                        mTargetPose.getRotation().getRadians() + Math.PI),
                fieldRelativeChassisSpeeds.omegaRadiansPerSecond);
    }

    @Override
    public void execute() {
        var hash = hashCode();
        if (kMaxVelocity.hasChanged(hash)
                || kMaxAcceleration.hasChanged(hash)
                || kMaxAngularVelocity.hasChanged(hash)
                || kMaxAngularAcceleration.hasChanged(hash)
                || kPositionKp.hasChanged(hash)
                || kPositionKi.hasChanged(hash)
                || kPositionKd.hasChanged(hash)
                || kRotationKp.hasChanged(hash)
                || kRotationKi.hasChanged(hash)
                || kRotationKd.hasChanged(hash)) {
            mPositionProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(
                    Math.min(kMaxVelocity.get(), mKinematicLimits.kMaxDriveVelocity),
                    Math.min(kMaxAcceleration.get(), mKinematicLimits.kMaxDriveAcceleration)));
            mRotationProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(
                    Math.min(kMaxAngularVelocity.get(), mKinematicLimits.kMaxDriveVelocity / kModuleRadius),
                    Math.min(kMaxAngularAcceleration.get(), mKinematicLimits.kMaxDriveAcceleration / kModuleRadius)));
            mPositionController.setPID(kPositionKp.get(), kPositionKi.get(), kPositionKd.get());
            mRotationController.setPID(kRotationKp.get(), kRotationKi.get(), kRotationKd.get());
        }

        var currentPose = PoseEstimator.getInstance().getPose2d();
        var translationToTarget = mTargetPose.getTranslation().minus(currentPose.getTranslation());
        var distanceToTarget = translationToTarget.getNorm();
        var headingToTarget = translationToTarget.getAngle();

        // Calculate directional velocity
        var positionPidOutput = mPositionController.calculate(distanceToTarget, mPositionState.position);
        var positionTargetState = new TrapezoidProfile.State(0.0, 0.0);
        mPositionState = mPositionProfile.calculate(Constants.kLoopPeriodSeconds, mPositionState, positionTargetState);
        var velocity = new Translation2d(-(mPositionState.velocity + positionPidOutput), headingToTarget);

        // Calculate rotational velocity
        var rotationPidOutput =
                mRotationController.calculate(currentPose.getRotation().getRadians(), mRotationState.position);
        var rotationTargetState =
                new TrapezoidProfile.State(mTargetPose.getRotation().getRadians(), 0.0);
        mRotationState = mRotationProfile.calculate(Constants.kLoopPeriodSeconds, mRotationState, rotationTargetState);
        var rotationalVelocity = mRotationState.velocity + rotationPidOutput;

        // Set drive outputs
        var atTargetPose = distanceToTarget < kPositionToleranceMeters.get()
                && GeometryUtil.isNear(
                        mTargetPose.getRotation(),
                        currentPose.getRotation(),
                        Rotation2d.fromRadians(kRotationToleranceRadians.get()));
        if (atTargetPose) {
            mDrive.stop();
            mSetpoint = currentPose;
        } else {
            mDrive.setVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(
                    velocity.getX(), velocity.getY(), rotationalVelocity, currentPose.getRotation()));
            mSetpoint = new Pose2d(
                    mTargetPose.getTranslation().minus(new Translation2d(mPositionState.position, headingToTarget)),
                    Rotation2d.fromRadians(mRotationState.position));
        }

        Logger.recordOutput(kLoggingPrefix + "PositionError", mPositionController.getPositionError());
        Logger.recordOutput(
                kLoggingPrefix + "RotationError", Rotation2d.fromRadians(mRotationController.getPositionError()));
        Logger.recordOutput(kLoggingPrefix + "Setpoint", mSetpoint);
        Logger.recordOutput(kLoggingPrefix + "TargetPose", mTargetPose);
    }

    @Override
    public void end(boolean interrupted) {
        mDrive.stop();
    }

    public boolean atTargetPose() {
        var currentPose = PoseEstimator.getInstance().getPose2d();
        var positionError =
                mTargetPose.getTranslation().minus(currentPose.getTranslation()).getNorm();
        return Util.inRange(positionError, kPositionToleranceMeters.get())
                && GeometryUtil.isNear(
                        mTargetPose.getRotation(),
                        currentPose.getRotation(),
                        Rotation2d.fromRadians(kRotationToleranceRadians.get()));
    }
}
