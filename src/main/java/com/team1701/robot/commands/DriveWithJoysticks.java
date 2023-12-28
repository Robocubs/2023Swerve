package com.team1701.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.team1701.lib.swerve.SwerveSetpointGenerator.KinematicLimits;
import com.team1701.robot.Constants;
import com.team1701.robot.subsystems.drive.Drive;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

public class DriveWithJoysticks extends Command {
    private final Drive mDrive;
    private final DoubleSupplier mThrottle;
    private final DoubleSupplier mStrafe;
    private final DoubleSupplier mRotation;
    private final Supplier<KinematicLimits> mKinematicLimits;

    DriveWithJoysticks(
            Drive drive,
            DoubleSupplier throttle,
            DoubleSupplier strafe,
            DoubleSupplier rotation,
            Supplier<KinematicLimits> kinematicLimits) {
        mDrive = drive;
        mThrottle = throttle;
        mStrafe = strafe;
        mRotation = rotation;
        mKinematicLimits = kinematicLimits;
        addRequirements(drive);
    }

    @Override
    public void execute() {
        var kinematicLimits = mKinematicLimits.get();
        mDrive.setKinematicLimits(kinematicLimits);

        var throttle = mThrottle.getAsDouble();
        var strafe = mStrafe.getAsDouble();
        var magnitude = Math.hypot(throttle, strafe);
        var rotation = MathUtil.applyDeadband(mRotation.getAsDouble(), Constants.Controls.kDriverDeadband);

        var rotationRadiansPerSecond = Math.copySign(rotation * rotation, rotation)
                * kinematicLimits.kMaxDriveVelocity
                / Constants.Drive.kModuleRadius;

        if (magnitude < Constants.Controls.kDriverDeadband) {
            mDrive.setVelocity(new ChassisSpeeds(0.0, 0.0, rotationRadiansPerSecond));
        } else {
            mDrive.setVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(
                    throttle * kinematicLimits.kMaxDriveVelocity,
                    strafe * kinematicLimits.kMaxDriveVelocity,
                    rotationRadiansPerSecond,
                    mDrive.getFieldRelativeHeading()));
        }
    }

    @Override
    public void end(boolean interrupted) {
        mDrive.stop();
    }
}
