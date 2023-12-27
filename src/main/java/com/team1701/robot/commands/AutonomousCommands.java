package com.team1701.robot.commands;

import com.team1701.lib.swerve.SwerveSetpointGenerator.KinematicLimits;
import com.team1701.lib.util.GeometryUtil;
import com.team1701.robot.Constants;
import com.team1701.robot.estimation.PoseEstimator;
import com.team1701.robot.subsystems.drive.Drive;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class AutonomousCommands {
    private final Drive mDrive;

    public AutonomousCommands(Drive drive) {
        mDrive = drive;
    }

    private Command resetPose(Pose2d pose) {
        return Commands.runOnce(() -> PoseEstimator.getInstance().setPose(pose));
    }

    private DriveToPose driveToPose(Pose2d pose) {
        return driveToPose(pose, Constants.Drive.kFastTrapezoidalKinematicLimits);
    }

    private DriveToPose driveToPose(Pose2d pose, KinematicLimits kinematicLimits) {
        return new DriveToPose(mDrive, pose, kinematicLimits);
    }

    public Command demo() {
        var driveToPose1 = driveToPose(new Pose2d(2.0, 1.0, GeometryUtil.kRotationIdentity));
        var driveToPose2 = driveToPose(new Pose2d(10.0, 1.0, GeometryUtil.kRotationHalfPi));
        var driveToPose3 = driveToPose(
                new Pose2d(2.0, 5.0, Rotation2d.fromRadians(-Math.PI * 2.0 / 3.0)),
                Constants.Drive.kSlowKinematicLimits);
        var driveToPose4 = driveToPose(new Pose2d(10.0, 5.0, Rotation2d.fromRadians(Math.PI * 2.0 / 3.0)));
        return Commands.sequence(
                resetPose(new Pose2d(6.0, 3.0, GeometryUtil.kRotationIdentity)),
                driveToPose1.until(driveToPose1::atTargetPose),
                driveToPose2.until(driveToPose2::atTargetPose),
                driveToPose3.until(driveToPose3::atTargetPose),
                driveToPose4);
    }
}
