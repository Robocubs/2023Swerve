package com.team1701.robot.subsystems.drive;

import java.util.Arrays;
import java.util.stream.Stream;

import com.team1701.lib.swerve.SwerveSetpoint;
import com.team1701.lib.swerve.SwerveSetpointGenerator;
import com.team1701.lib.swerve.SwerveSetpointGenerator.KinematicLimits;
import com.team1701.lib.util.GeometryUtil;
import com.team1701.robot.Configuration;
import com.team1701.robot.Configuration.Mode;
import com.team1701.robot.Constants;
import com.team1701.robot.estimation.PoseEstimator;
import com.team1701.robot.loops.Loop;
import com.team1701.robot.loops.Looper;
import com.team1701.robot.subsystems.Subsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.littletonrobotics.junction.Logger;

public class Drive extends Subsystem {
    private static Drive mInstance;

    private final GyroInputsAutoLogged mGyroInputs = new GyroInputsAutoLogged();
    private final GyroIO mGyroIO;
    private final SwerveModule[] mModules;
    private final SwerveSetpointGenerator mSetpointGenerator = new SwerveSetpointGenerator(Constants.Drive.kKinematics);

    private KinematicLimits mKinematicLimits = Constants.Drive.kFastKinematicLimits;
    private ChassisSpeeds mDesiredChassisSpeeds = new ChassisSpeeds();
    private SwerveSetpoint mDesiredSetpoint = new SwerveSetpoint(Constants.Drive.kNumModules);
    private SwerveModuleState[] mMeasuredModuleStates;
    private SwerveModulePosition[] mMeasuredModulePositions;
    private Rotation2d mFieldRelativeHeading;
    private double mYawOffset;

    public static Drive getInstance() {
        if (mInstance == null) {
            mInstance = new Drive();
        }

        return mInstance;
    }

    private Drive() {
        GyroIO gyroIO = null;
        SwerveModuleIO[] moduleIOs = null;
        if (Configuration.getMode() != Mode.REPLAY) {
            switch (Configuration.getRobot()) {
                case SWERVE_BOT:
                    gyroIO = new GyroIOPigeon2(10);
                    moduleIOs = new SwerveModuleIO[] {
                        new SwerveModuleIOSparkMax(
                                10,
                                11,
                                0,
                                true,
                                Constants.Drive.kMotorsInverted,
                                Constants.Drive.kSteerReduction,
                                Rotation2d.fromRadians(-4.54)),
                        new SwerveModuleIOSparkMax(
                                12,
                                13,
                                1,
                                true,
                                Constants.Drive.kMotorsInverted,
                                Constants.Drive.kSteerReduction,
                                Rotation2d.fromRadians(-4.28)),
                        new SwerveModuleIOSparkMax(
                                16,
                                17,
                                3,
                                true,
                                Constants.Drive.kMotorsInverted,
                                Constants.Drive.kSteerReduction,
                                Rotation2d.fromRadians(-0.18)),
                        new SwerveModuleIOSparkMax(
                                14,
                                15,
                                2,
                                true,
                                Constants.Drive.kMotorsInverted,
                                Constants.Drive.kSteerReduction,
                                Rotation2d.fromRadians(-2.00)),
                    };
                    break;
                case SIMULATION_BOT:
                    gyroIO = new GyroIOSim(() -> Constants.Drive.kKinematics.toChassisSpeeds(mMeasuredModuleStates));
                    moduleIOs = new SwerveModuleIO[] {
                        new SwerveModuleIOSim() {},
                        new SwerveModuleIOSim() {},
                        new SwerveModuleIOSim() {},
                        new SwerveModuleIOSim() {},
                    };
                    break;
                default:
                    break;
            }
        }

        if (gyroIO == null) {
            gyroIO = new GyroIO() {};
        }

        if (moduleIOs == null) {
            moduleIOs = new SwerveModuleIO[Constants.Drive.kNumModules];
            Arrays.setAll(moduleIOs, i -> new SwerveModuleIO() {});
        }

        mGyroIO = gyroIO;
        mModules = new SwerveModule[moduleIOs.length];
        for (int i = 0; i < mModules.length; i++) {
            mModules[i] = new SwerveModule(i, moduleIOs[i]);
            mModules[i].setSteerBrakeMode(false);
            mModules[i].setDriveBrakeMode(false);
        }

        updateInputs();
        zeroModules();
        setModuleSetpointsFromMeasured();
    }

    public void setVelocity(ChassisSpeeds chassisSpeeds) {
        mDesiredChassisSpeeds = chassisSpeeds;
    }

    public Rotation2d getFieldRelativeRotation() {
        return mFieldRelativeHeading;
    }

    public void zeroGyroscope() {
        zeroGyroscope(GeometryUtil.kRotationIdentity);
    }

    public void zeroGyroscope(Rotation2d rotation) {
        var normalizedRadians = MathUtil.angleModulus(rotation.getRadians());
        mYawOffset = mGyroInputs.yawPositionRad - normalizedRadians;
        mFieldRelativeHeading = new Rotation2d(normalizedRadians);
    }

    public void zeroModules() {
        for (var module : mModules) {
            module.zeroSteeringMotor();
        }

        mMeasuredModuleStates = Stream.of(mModules).map(SwerveModule::getState).toArray(SwerveModuleState[]::new);
        mMeasuredModulePositions =
                Stream.of(mModules).map(SwerveModule::getPosition).toArray(SwerveModulePosition[]::new);

        PoseEstimator.getInstance()
                .resetPosition(
                        new Rotation2d(mGyroInputs.yawPositionRad),
                        mMeasuredModulePositions,
                        PoseEstimator.getInstance().get());
    }

    private void setModuleSetpointsFromMeasured() {
        mDesiredSetpoint.mChassisSpeeds =
                Constants.Drive.kKinematics.toChassisSpeedWheelConstraints(mDesiredSetpoint.mModuleStates);
        for (var i = 0; i < mModules.length; ++i) {
            mDesiredSetpoint.mModuleStates[i] = mMeasuredModuleStates[i];
        }
    }

    private void updateDesiredStates() {
        // Calculate the output chassis speeds to make the desired move in the next loop
        var desiredRelativePose = new Pose2d(
                mDesiredChassisSpeeds.vxMetersPerSecond * Constants.kLoopPeriodSeconds,
                mDesiredChassisSpeeds.vyMetersPerSecond * Constants.kLoopPeriodSeconds,
                Rotation2d.fromRadians(mDesiredChassisSpeeds.omegaRadiansPerSecond * Constants.kLoopPeriodSeconds));
        var twistToDesiredPose = GeometryUtil.kPoseIdentity.log(desiredRelativePose);
        var outputChassisSpeeds = new ChassisSpeeds(
                twistToDesiredPose.dx / Constants.kLoopPeriodSeconds,
                twistToDesiredPose.dy / Constants.kLoopPeriodSeconds,
                twistToDesiredPose.dtheta / Constants.kLoopPeriodSeconds);
        mDesiredSetpoint = mSetpointGenerator.generateSetpoint(
                mKinematicLimits, mDesiredSetpoint, outputChassisSpeeds, Constants.kLoopPeriodSeconds);
        for (var i = 0; i < mDesiredSetpoint.mModuleStates.length; i++) {
            mModules[i].setState(mDesiredSetpoint.mModuleStates[i]);
        }
    }

    @Override
    public void registerEnabledLoops(Looper looper) {
        looper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                setModuleSetpointsFromMeasured();
                setVelocity(new ChassisSpeeds());
                mKinematicLimits = Constants.Drive.kUncappedKinematicLimits;
            }

            @Override
            public void onLoop(double timestamp) {
                updateDesiredStates();
            }

            @Override
            public void onStop(double timestamp) {
                stop();
            }

            @Override
            public String getDisplayName() {
                return Drive.class.getSimpleName();
            }
        });
    }

    @Override
    public void readPeriodicInputs() {
        updateInputs();
        PoseEstimator.getInstance().update(new Rotation2d(mGyroInputs.yawPositionRad), mMeasuredModulePositions);
    }

    private void updateInputs() {
        for (var module : mModules) {
            module.readPeriodicInputs();
        }

        mMeasuredModuleStates = Stream.of(mModules).map(SwerveModule::getState).toArray(SwerveModuleState[]::new);
        mMeasuredModulePositions =
                Stream.of(mModules).map(SwerveModule::getPosition).toArray(SwerveModulePosition[]::new);

        mGyroIO.updateInputs(mGyroInputs);
        Logger.getInstance().processInputs("Drive/Gyro", mGyroInputs);
        mFieldRelativeHeading = new Rotation2d(mGyroInputs.yawPositionRad - mYawOffset);
    }

    @Override
    public void writePeriodicOutputs() {
        for (var module : mModules) {
            module.writePeriodicOutputs();
        }
    }

    @Override
    public void stop() {
        mDesiredChassisSpeeds = new ChassisSpeeds();
    }

    @Override
    public void outputTelemetry() {
        Logger.getInstance().recordOutput("Drive/Gyro/FieldRelativeRad", mFieldRelativeHeading.getRadians());
        Logger.getInstance().recordOutput("Drive/Gyro/FieldRelativeDeg", mFieldRelativeHeading.getDegrees());
        Logger.getInstance().recordOutput("Drive/MeasuredStates", mMeasuredModuleStates);
        Logger.getInstance().recordOutput("Drive/DesiredStates", mDesiredSetpoint.mModuleStates);

        for (var module : mModules) {
            module.outputTelemetry();
        }
    }
}
