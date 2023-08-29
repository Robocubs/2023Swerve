package com.team1701.robot;

import com.team1701.lib.swerve.ExtendedSwerveDriveKinematics;
import com.team1701.lib.swerve.SwerveSetpointGenerator.KinematicLimits;
import com.team1701.lib.util.LoggedTunableNumber;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public final class Constants {
    public static final double kLoopPeriodSeconds = 0.02;

    public static final class Controls {
        public static final double kDriverMagDeadZone = 0.09;
        public static final double kDriverXDeadZone = 0.09;
        public static final double kDriverYDeadZone = 0.09;
    }

    public static final class Motors {
        public static final double kMaxNeoRPM = 5676;
    }

    public static final class Drive {
        protected static final double kL1DriveReduction = (14.0 / 50.0) * (25.0 / 19.0) * (15.0 / 45.0);
        protected static final double kL2DriveReduction = (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0);
        protected static final double kL3DriveReduction = (14.0 / 50.0) * (28.0 / 16.0) * (15.0 / 45.0);

        protected static final double kMk4SteerReduction = 1.0 / 12.8;
        protected static final double kMk4iSteerReduction = 7.0 / 150.0;

        public static final double kWheelRadiusMeters;
        public static final double kMaxVelocityMetersPerSecond;
        public static final double kMaxAngularVelocityRadiansPerSecond;
        public static final boolean kMotorsInverted;
        public static final double kDriveReduction;
        public static final double kSteerReduction;

        public static final int kNumModules;
        public static final ExtendedSwerveDriveKinematics kKinematics;
        public static final KinematicLimits kUncappedKinematicLimits;
        public static final KinematicLimits kFastKinematicLimits;
        public static final KinematicLimits kSlowKinematicLimits;

        public static final LoggedTunableNumber kDriveKf = new LoggedTunableNumber("Drive/Module/DriveKf");
        public static final LoggedTunableNumber kDriveKp = new LoggedTunableNumber("Drive/Module/DriveKp");
        public static final LoggedTunableNumber kDriveKd = new LoggedTunableNumber("Drive/Module/DriveKd");
        public static final LoggedTunableNumber kSteerKp = new LoggedTunableNumber("Drive/Module/SteerKp");
        public static final LoggedTunableNumber kSteerKd = new LoggedTunableNumber("Drive/Module/SteerKd");

        static {
            double driveMotorMaxRPM;
            double driveTrackWidthMeters;
            double driveWheelbaseMeters;

            switch (Configuration.getRobot()) {
                case SWERVE_BOT:
                    throw new UnsupportedOperationException("Not implemented yet");
                case SIMULATION_BOT:
                    kWheelRadiusMeters = Units.inchesToMeters(2);
                    driveTrackWidthMeters = 0.5;
                    driveWheelbaseMeters = 0.5;
                    driveMotorMaxRPM = Constants.Motors.kMaxNeoRPM;
                    kDriveReduction = kL3DriveReduction;
                    kSteerReduction = kMk4iSteerReduction;
                    kMotorsInverted = true;
                    kDriveKf.initDefault(0.12);
                    kDriveKp.initDefault(1.8);
                    kDriveKd.initDefault(0);
                    kSteerKp.initDefault(23.0);
                    kSteerKd.initDefault(0);
                    break;
                default:
                    throw new UnsupportedOperationException("No drive configuration for " + Configuration.getRobot());
            }

            kMaxVelocityMetersPerSecond =
                    driveMotorMaxRPM / 60.0 * (2 * Math.PI) * kDriveReduction * kWheelRadiusMeters;
            kMaxAngularVelocityRadiansPerSecond =
                    kMaxVelocityMetersPerSecond / Math.hypot(driveTrackWidthMeters / 2.0, driveWheelbaseMeters / 2.0);

            kKinematics = new ExtendedSwerveDriveKinematics(
                    // Front left
                    new Translation2d(driveTrackWidthMeters / 2.0, driveWheelbaseMeters / 2.0),
                    // Front right
                    new Translation2d(driveTrackWidthMeters / 2.0, -driveWheelbaseMeters / 2.0),
                    // Back left
                    new Translation2d(-driveTrackWidthMeters / 2.0, driveWheelbaseMeters / 2.0),
                    // Back right
                    new Translation2d(-driveTrackWidthMeters / 2.0, -driveWheelbaseMeters / 2.0));

            kNumModules = kKinematics.getNumModules();

            kUncappedKinematicLimits =
                    new KinematicLimits(kMaxVelocityMetersPerSecond, Double.MAX_VALUE, Double.MAX_VALUE);
            kFastKinematicLimits = new KinematicLimits(
                    kMaxVelocityMetersPerSecond, kMaxVelocityMetersPerSecond / 0.2, Units.degreesToRadians(1000.0));
            kSlowKinematicLimits = new KinematicLimits(
                    kMaxVelocityMetersPerSecond * 0.5,
                    kMaxVelocityMetersPerSecond * 0.5 / 0.2,
                    Units.degreesToRadians(750.0));
        }
    }
}
