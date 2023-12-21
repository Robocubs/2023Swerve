package com.team1701.robot.subsystems.drive;

import java.util.Arrays;
import java.util.stream.Stream;

import com.team1701.lib.drivers.encoders.EncoderIO;
import com.team1701.lib.drivers.encoders.EncoderIOAnalog;
import com.team1701.lib.drivers.encoders.EncoderIOSim;
import com.team1701.lib.drivers.gyros.GyroIO;
import com.team1701.lib.drivers.gyros.GyroIOPigeon2;
import com.team1701.lib.drivers.gyros.GyroIOSim;
import com.team1701.lib.drivers.gyros.GyroInputsAutoLogged;
import com.team1701.lib.drivers.motors.MotorIO;
import com.team1701.lib.drivers.motors.MotorIOSim;
import com.team1701.lib.swerve.SwerveSetpoint;
import com.team1701.lib.swerve.SwerveSetpointGenerator;
import com.team1701.lib.swerve.SwerveSetpointGenerator.KinematicLimits;
import com.team1701.lib.util.GeometryUtil;
import com.team1701.lib.util.SignalSamplingThread;
import com.team1701.robot.Configuration;
import com.team1701.robot.Configuration.Mode;
import com.team1701.robot.Constants;
import com.team1701.robot.estimation.PoseEstimator;
import com.team1701.robot.loops.Loop;
import com.team1701.robot.loops.Looper;
import com.team1701.robot.subsystems.Subsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.Logger;

public class Drive extends Subsystem {
    private static Drive mInstance;

    private final GyroInputsAutoLogged mGyroInputs = new GyroInputsAutoLogged();
    private final GyroIO mGyroIO;
    private final SwerveModule[] mModules;
    private final SwerveSetpointGenerator mSetpointGenerator = new SwerveSetpointGenerator(Constants.Drive.kKinematics);
    private final SignalSamplingThread mOdometryThread =
            new SignalSamplingThread("OdometryThread", 1 / Constants.Drive.kOdometryFrequency);

    private KinematicLimits mKinematicLimits = Constants.Drive.kFastKinematicLimits;
    private ChassisSpeeds mDesiredChassisSpeeds = new ChassisSpeeds();
    private SwerveSetpoint mDesiredSetpoint = new SwerveSetpoint(Constants.Drive.kNumModules);
    private SwerveModuleState[] mMeasuredModuleStates;
    private SwerveModulePosition[] mMeasuredModulePositions;
    private Rotation2d mFieldRelativeHeading = GeometryUtil.kRotationIdentity;
    private Rotation2d mYawOffset = GeometryUtil.kRotationIdentity;
    private double mPreviousOdometryTimestamp = 0.0;

    public static Drive getInstance() {
        if (mInstance == null) {
            mInstance = new Drive();
        }

        return mInstance;
    }

    private Drive() {
        mMeasuredModuleStates = new SwerveModuleState[Constants.Drive.kNumModules];
        Arrays.setAll(mMeasuredModuleStates, i -> new SwerveModuleState());

        mMeasuredModulePositions = new SwerveModulePosition[Constants.Drive.kNumModules];
        Arrays.setAll(mMeasuredModulePositions, i -> new SwerveModulePosition());

        GyroIO gyroIO = null;
        SwerveModule[] modules = null;
        if (Configuration.getMode() != Mode.REPLAY) {
            switch (Configuration.getRobot()) {
                case SWERVE_BOT:
                    gyroIO = new GyroIOPigeon2(10);
                    var configurations = new SwerveModuleConfiguration[] {
                        new SwerveModuleConfiguration(10, 11, 0),
                        new SwerveModuleConfiguration(12, 13, 1),
                        new SwerveModuleConfiguration(16, 17, 3),
                        new SwerveModuleConfiguration(14, 15, 2),
                    };

                    modules = new SwerveModule[configurations.length];
                    for (var i = 0; i < modules.length; i++) {
                        modules[i] = new SwerveModule(
                                i,
                                DriveMotorFactory.createDriveMotorIOSparkMax(
                                        configurations[i].driveId, Constants.Drive.kMotorsInverted),
                                DriveMotorFactory.createSteerMotorIOSparkMax(
                                        configurations[i].steerId, Constants.Drive.kMotorsInverted),
                                new EncoderIOAnalog(configurations[i].steerId),
                                mOdometryThread);
                    }

                    break;
                case SIMULATION_BOT:
                    gyroIO = new GyroIOSim(
                            () -> Constants.Drive.kKinematics.toChassisSpeeds(mMeasuredModuleStates),
                            Constants.kLoopPeriodSeconds);

                    final var simModules = new SwerveModule[Constants.Drive.kNumModules];
                    modules = simModules;
                    for (var i = 0; i < simModules.length; i++) {
                        final var index = i;
                        var driveMotor = new MotorIOSim(
                                DCMotor.getNEO(1), Constants.Drive.kDriveReduction, 0.14, Constants.kLoopPeriodSeconds);
                        var steerMotor = new MotorIOSim(
                                DCMotor.getNEO(1),
                                Constants.Drive.kSteerReduction,
                                0.004,
                                Constants.kLoopPeriodSeconds);
                        steerMotor.enableContinuousInput(0, 2 * Math.PI / Constants.Drive.kSteerReduction);
                        var steerEncoder = new EncoderIOSim(() -> simModules[index].getPosition().angle);
                        simModules[index] =
                                new SwerveModule(index, driveMotor, steerMotor, steerEncoder, mOdometryThread);
                    }

                    break;
                default:
                    break;
            }
        }

        if (gyroIO == null) {
            gyroIO = new GyroIO() {};
        }

        gyroIO.enableYawSampling(mOdometryThread);

        if (modules == null) {
            modules = new SwerveModule[Constants.Drive.kNumModules];
            Arrays.setAll(
                    modules,
                    i -> new SwerveModule(i, new MotorIO() {}, new MotorIO() {}, new EncoderIO() {}, mOdometryThread));
        }

        for (var module : modules) {
            module.setSteerBrakeMode(false);
            module.setDriveBrakeMode(false);
        }

        mGyroIO = gyroIO;
        mModules = modules;

        updateInputs();
        zeroModules();
        setModuleSetpointsFromMeasured();
    }

    public SignalSamplingThread getOdometryThread() {
        return mOdometryThread;
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
        mYawOffset = mGyroInputs.yaw.minus(rotation);
        mFieldRelativeHeading = rotation;
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
                        mGyroInputs.yaw,
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
        updateOdometry();
    }

    private void updateInputs() {
        var odometryLock = mOdometryThread.getLock();
        odometryLock.lock();
        try {
            for (var module : mModules) {
                module.readPeriodicInputs();
            }

            mGyroIO.updateInputs(mGyroInputs);
        } finally {
            odometryLock.unlock();
        }

        Logger.processInputs("Drive/Gyro", mGyroInputs);

        mMeasuredModuleStates = Stream.of(mModules).map(SwerveModule::getState).toArray(SwerveModuleState[]::new);
        mMeasuredModulePositions =
                Stream.of(mModules).map(SwerveModule::getPosition).toArray(SwerveModulePosition[]::new);

        mFieldRelativeHeading = mGyroInputs.yaw.minus(mYawOffset);
    }

    private void updateOdometry() {
        var modulePositionSamples =
                Stream.of(mModules).map(SwerveModule::getPositionSamples).toArray(SwerveModulePosition[][]::new);
        var yawSamples = mGyroInputs.yawSamples;

        var minSamples = Stream.of(modulePositionSamples)
                .mapToInt(samples -> samples.length)
                .min()
                .orElse(0);
        minSamples = Math.min(minSamples, yawSamples.length);

        var timestamp = Timer.getFPGATimestamp();
        var sampleDt = (timestamp - mPreviousOdometryTimestamp) / (minSamples + 1);
        var sampleTime = mPreviousOdometryTimestamp + sampleDt;
        for (var i = 0; i < minSamples; i++) {
            var modulePositions = new SwerveModulePosition[modulePositionSamples.length];
            for (var j = 0; j < modulePositionSamples.length; j++) {
                modulePositions[j] = modulePositionSamples[j][i];
            }

            PoseEstimator.getInstance().updateWithTime(sampleTime, yawSamples[i], modulePositions);
            sampleTime += sampleDt;
        }

        PoseEstimator.getInstance().update(mGyroInputs.yaw, mMeasuredModulePositions);

        mPreviousOdometryTimestamp = timestamp;
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
        Logger.recordOutput("Drive/Gyro/FieldRelativeRad", mFieldRelativeHeading.getRadians());
        Logger.recordOutput("Drive/Gyro/FieldRelativeDeg", mFieldRelativeHeading.getDegrees());
        Logger.recordOutput("Drive/MeasuredStates", mMeasuredModuleStates);
        Logger.recordOutput("Drive/DesiredStates", mDesiredSetpoint.mModuleStates);

        for (var module : mModules) {
            module.outputTelemetry();
        }
    }
}
