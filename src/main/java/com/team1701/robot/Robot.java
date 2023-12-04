// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team1701.robot;

import com.team1701.robot.Configuration.Mode;
import com.team1701.robot.estimation.PoseEstimator;
import com.team1701.robot.loops.LoopRunner;
import com.team1701.robot.subsystems.drive.Drive;
import com.team1701.robot.subsystems.vision.Vision;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
    private final LoopRunner mEnabledLooper = new LoopRunner("enabled");
    private final LoopRunner mDisabledLooper = new LoopRunner("disabled");
    private final ControllerManager mControllerManager = ControllerManager.getInstance();
    private final SubsystemManager mSubsystemManager = SubsystemManager.getInstance();
    private final Drive mDrive = Drive.getInstance();
    private final Vision mVision = Vision.getInstance();
    private final PoseEstimator mPoseEstimator = PoseEstimator.getInstance();

    @Override
    public void robotInit() {
        initializeAdvantageKit();

        mSubsystemManager.setSubsystems(mDrive, mVision);
        mSubsystemManager.registerEnabledLoops(mEnabledLooper);
        mSubsystemManager.registerDisabledLoops(mDisabledLooper);
        mSubsystemManager.outputTelemetry();

        createJoystickHandlers();
    }

    private void initializeAdvantageKit() {
        var logger = Logger.getInstance();

        // Record metadata
        logger.recordMetadata("RuntimeType", getRuntimeType().toString());
        logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
        logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
        logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
        logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
        logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
        switch (BuildConstants.DIRTY) {
            case 0:
                logger.recordMetadata("GitDirty", "All changes committed");
                break;
            case 1:
                logger.recordMetadata("GitDirty", "Uncomitted changes");
                break;
            default:
                logger.recordMetadata("GitDirty", "Unknown");
                break;
        }

        // Set up data receivers & replay source
        switch (Configuration.getMode()) {
            case REAL:
                logger.addDataReceiver(new WPILOGWriter("/media/sda1/"));
                logger.addDataReceiver(new NT4Publisher());
                break;
            case SIMULATION:
                logger.addDataReceiver(new NT4Publisher());
                break;
            case REPLAY:
                var logPath = LogFileUtil.findReplayLog();
                logger.setReplaySource(new WPILOGReader(logPath));
                logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
                break;
        }

        // Start AdvantageKit logger
        setUseTiming(Configuration.getMode() != Mode.REPLAY);
        logger.start();
    }

    private void createJoystickHandlers() {
        var driver = mControllerManager.getDriverJoystick();

        driver.onButtonPressed(ControllerManager.kXBOXButtonX, () -> {
            mDrive.zeroGyroscope();
        });
    }

    @Override
    public void robotPeriodic() {
        mEnabledLooper.loop();
        mDisabledLooper.loop();
        mSubsystemManager.outputTelemetry();
        mEnabledLooper.outputTelemetry();
        mDisabledLooper.outputTelemetry();
        mPoseEstimator.outputTelemetry();
    }

    @Override
    public void autonomousInit() {
        mDisabledLooper.stop();
        mEnabledLooper.start();
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {
        mDisabledLooper.stop();
        mControllerManager.resetHandlers();
        mEnabledLooper.start();
    }

    @Override
    public void teleopPeriodic() {
        mControllerManager.invokeHandlers();
        drive();
    }

    private void drive() {
        var throttle = -mControllerManager.getDriverJoystick().getY();
        var strafe = -mControllerManager.getDriverJoystick().getX();
        var rot = -mControllerManager.getDriverJoystick().getZWithDeadZone();
        var mag = Math.hypot(throttle, strafe);

        if (mag < Constants.Controls.kDriverMagDeadZone) {
            mDrive.setVelocity(new ChassisSpeeds(0, 0, rot * Constants.Drive.kMaxAngularVelocityRadiansPerSecond));
        } else {
            mDrive.setVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(
                    throttle * Constants.Drive.kMaxVelocityMetersPerSecond,
                    strafe * Constants.Drive.kMaxVelocityMetersPerSecond,
                    rot * Constants.Drive.kMaxAngularVelocityRadiansPerSecond,
                    mDrive.getFieldRelativeRotation()));
        }
    }

    @Override
    public void disabledInit() {

        mEnabledLooper.stop();
        mDisabledLooper.start();
    }

    @Override
    public void disabledPeriodic() {}

    @Override
    public void testInit() {}

    @Override
    public void testPeriodic() {}

    @Override
    public void simulationInit() {}

    @Override
    public void simulationPeriodic() {}
}
