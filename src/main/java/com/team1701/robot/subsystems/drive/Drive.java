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
import com.team1701.lib.util.TimeLockedBoolean;
import com.team1701.lib.util.Util;
import com.team1701.robot.Configuration;
import com.team1701.robot.Configuration.Mode;
import com.team1701.robot.Constants;
import com.team1701.robot.estimation.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {
    private static Drive mInstance = null;

    private final PoseEstimator mPoseEstimator = PoseEstimator.getInstance();
    private final GyroInputsAutoLogged mGyroInputs = new GyroInputsAutoLogged();
    private final GyroIO mGyroIO;
    private final SwerveModule[] mModules;
    private final SwerveSetpointGenerator mSetpointGenerator = new SwerveSetpointGenerator(Constants.Drive.kKinematics);
    private final SignalSamplingThread mOdometryThread =
            new SignalSamplingThread("OdometryThread", 1 / Constants.Drive.kOdometryFrequency);

    private KinematicLimits mKinematicLimits = Constants.Drive.kFastKinematicLimits;
    private ChassisSpeeds mDesiredChassisSpeeds = new ChassisSpeeds();
    private SwerveSetpoint mPreviousSetpoint = new SwerveSetpoint(Constants.Drive.kNumModules);
    private SwerveModulePosition[] mMeasuredModulePositions;
    private Rotation2d mFieldRelativeHeading = GeometryUtil.kRotationIdentity;
    private Rotation2d mYawOffset = GeometryUtil.kRotationIdentity;
    private double mPreviousOdometryTimestamp = 0.0;
    private TimeLockedBoolean mWasMovingRecently = new TimeLockedBoolean(1.0, 0.0, false, false);

    @AutoLogOutput(key = "Drive/MeasuredStates")
    private SwerveModuleState[] mMeasuredModuleStates;

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
        SwerveModuleIO[] moduleIOs = null;
        if (Configuration.getMode() != Mode.REPLAY) {
            switch (Configuration.getRobot()) {
                case SWERVE_BOT:
                    gyroIO = new GyroIOPigeon2(10);
                    moduleIOs = Stream.of(new SwerveModuleConfiguration[] {
                                new SwerveModuleConfiguration(10, 11, 0),
                                new SwerveModuleConfiguration(12, 13, 1),
                                new SwerveModuleConfiguration(16, 17, 3),
                                new SwerveModuleConfiguration(14, 15, 2),
                            })
                            .map(config -> new SwerveModuleIO(
                                    DriveMotorFactory.createDriveMotorIOSparkMax(
                                            config.driveId, Constants.Drive.kMotorsInverted),
                                    DriveMotorFactory.createSteerMotorIOSparkMax(
                                            config.steerId, Constants.Drive.kMotorsInverted),
                                    new EncoderIOAnalog(config.steerId)))
                            .toArray(SwerveModuleIO[]::new);
                    break;
                case SIMULATION_BOT:
                    gyroIO = new GyroIOSim(
                            () -> Constants.Drive.kKinematics.toChassisSpeeds(mMeasuredModuleStates),
                            Constants.kLoopPeriodSeconds);
                    moduleIOs = new SwerveModuleIO[Constants.Drive.kNumModules];
                    for (var i = 0; i < moduleIOs.length; i++) {
                        var index = i;
                        var driveMotor = new MotorIOSim(
                                DCMotor.getNEO(1), Constants.Drive.kDriveReduction, 0.14, Constants.kLoopPeriodSeconds);
                        var steerMotor = new MotorIOSim(
                                DCMotor.getNEO(1),
                                Constants.Drive.kSteerReduction,
                                0.004,
                                Constants.kLoopPeriodSeconds);
                        steerMotor.enableContinuousInput(0, 2 * Math.PI / Constants.Drive.kSteerReduction);
                        var steerEncoder = new EncoderIOSim(() -> mMeasuredModulePositions[index].angle);
                        moduleIOs[i] = new SwerveModuleIO(driveMotor, steerMotor, steerEncoder);
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

        if (moduleIOs == null) {
            moduleIOs = new SwerveModuleIO[Constants.Drive.kNumModules];
            Arrays.setAll(moduleIOs, i -> new SwerveModuleIO(new MotorIO() {}, new MotorIO() {}, new EncoderIO() {}));
        }

        mGyroIO = gyroIO;
        mModules = new SwerveModule[moduleIOs.length];
        for (var i = 0; i < mModules.length; i++) {
            mModules[i] = new SwerveModule(i, moduleIOs[i], mOdometryThread);
        }

        for (var module : mModules) {
            module.setSteerBrakeMode(false);
            module.setDriveBrakeMode(false);
        }

        updateInputs();
        zeroModules();
    }

    @Override
    public void periodic() {
        updateInputs();
        updateOdometry();
        updateDesiredStates();
    }

    private void updateInputs() {
        var odometryLock = mOdometryThread.getLock();
        odometryLock.lock();
        try {
            for (var module : mModules) {
                module.updateInputs();
            }

            mGyroIO.updateInputs(mGyroInputs);
        } finally {
            odometryLock.unlock();
        }

        Logger.processInputs("Drive/Gyro", mGyroInputs);

        for (var module : mModules) {
            module.periodic();
        }

        mMeasuredModuleStates = Stream.of(mModules).map(SwerveModule::getState).toArray(SwerveModuleState[]::new);
        mMeasuredModulePositions =
                Stream.of(mModules).map(SwerveModule::getPosition).toArray(SwerveModulePosition[]::new);

        mFieldRelativeHeading = mGyroInputs.yaw.minus(mYawOffset);

        var timestamp = Timer.getFPGATimestamp();
        for (var state : mMeasuredModuleStates) {
            mWasMovingRecently.update(!Util.epsilonEquals(state.speedMetersPerSecond, 0.0, 0.02), timestamp);
        }
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

            mPoseEstimator.updateWithTime(sampleTime, yawSamples[i], modulePositions);
            sampleTime += sampleDt;
        }

        mPoseEstimator.update(mGyroInputs.yaw, mMeasuredModulePositions);

        mPreviousOdometryTimestamp = timestamp;
    }

    private void updateDesiredStates() {
        var desiredChassisSpeedIsZero = Util.epsilonEquals(mDesiredChassisSpeeds.vxMetersPerSecond, 0.0)
                && Util.epsilonEquals(mDesiredChassisSpeeds.vyMetersPerSecond, 0.0)
                && Util.epsilonEquals(mDesiredChassisSpeeds.omegaRadiansPerSecond, 0.0);
        var wasMovingRecently = mWasMovingRecently.update(!desiredChassisSpeedIsZero, Timer.getFPGATimestamp());
        if (DriverStation.isDisabled() || !wasMovingRecently) {
            for (var module : mModules) {
                module.stop();
            }

            mPreviousSetpoint = new SwerveSetpoint(Constants.Drive.kKinematics, mMeasuredModuleStates);
            Logger.recordOutput("Drive/DesiredStates", new SwerveModuleState[] {});
            return;
        }

        var desiredRelativePose = new Pose2d(
                mDesiredChassisSpeeds.vxMetersPerSecond * Constants.kLoopPeriodSeconds,
                mDesiredChassisSpeeds.vyMetersPerSecond * Constants.kLoopPeriodSeconds,
                Rotation2d.fromRadians(mDesiredChassisSpeeds.omegaRadiansPerSecond * Constants.kLoopPeriodSeconds));
        var twistToDesiredPose = GeometryUtil.kPoseIdentity.log(desiredRelativePose);
        var outputChassisSpeeds = new ChassisSpeeds(
                twistToDesiredPose.dx / Constants.kLoopPeriodSeconds,
                twistToDesiredPose.dy / Constants.kLoopPeriodSeconds,
                twistToDesiredPose.dtheta / Constants.kLoopPeriodSeconds);
        var desiredSetpoint = mSetpointGenerator.generateSetpoint(
                mKinematicLimits, mPreviousSetpoint, outputChassisSpeeds, Constants.kLoopPeriodSeconds);
        for (var i = 0; i < mModules.length; i++) {
            mModules[i].setState(desiredSetpoint.moduleStates[i]);
        }

        mPreviousSetpoint = desiredSetpoint;
        Logger.recordOutput("Drive/DesiredStates", desiredSetpoint.moduleStates);
    }

    public void setVelocity(ChassisSpeeds chassisSpeeds) {
        mDesiredChassisSpeeds = chassisSpeeds;
    }

    @AutoLogOutput()
    public Rotation2d getFieldRelativeHeading() {
        return mFieldRelativeHeading;
    }

    @AutoLogOutput
    public double getSpeedMetersPerSecond() {
        var chassisSpeeds = Constants.Drive.kKinematics.toChassisSpeeds(mMeasuredModuleStates);
        return Math.hypot(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);
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

        mPoseEstimator.resetPosition(mGyroInputs.yaw, mMeasuredModulePositions, mPoseEstimator.getPose2d());
    }

    public void stop() {
        setVelocity(new ChassisSpeeds());
    }
}
