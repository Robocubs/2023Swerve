package com.team1701.robot.commands;

import java.util.function.DoubleSupplier;

import com.team1701.robot.Constants;
import com.team1701.robot.subsystems.drive.Drive;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

public class JoystickDriveCommand extends Command {
    private final Drive mDrive;
    private final DoubleSupplier mThrottle;
    private final DoubleSupplier mStrafe;
    private final DoubleSupplier mRotation;

    public JoystickDriveCommand(DoubleSupplier throttle, DoubleSupplier strafe, DoubleSupplier rotation) {
        mDrive = Drive.getInstance();
        mThrottle = throttle;
        mStrafe = strafe;
        mRotation = rotation;
        addRequirements(mDrive);
    }

    @Override
    public void execute() {
        var throttle = mThrottle.getAsDouble();
        var strafe = mStrafe.getAsDouble();
        var rotation = MathUtil.applyDeadband(mRotation.getAsDouble(), Constants.Controls.kDriverDeadband);
        var magnitude = Math.hypot(throttle, strafe);

        if (magnitude < Constants.Controls.kDriverDeadband) {
            mDrive.setVelocity(
                    new ChassisSpeeds(0.0, 0.0, rotation * Constants.Drive.kMaxAngularVelocityRadiansPerSecond));
        } else {
            mDrive.setVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(
                    throttle * Constants.Drive.kMaxVelocityMetersPerSecond,
                    strafe * Constants.Drive.kMaxVelocityMetersPerSecond,
                    rotation * Constants.Drive.kMaxAngularVelocityRadiansPerSecond,
                    mDrive.getFieldRelativeHeading()));
        }
    }
}
