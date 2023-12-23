// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team1701.robot;

import com.team1701.robot.Configuration.Mode;
import com.team1701.robot.commands.JoystickDriveCommand;
import com.team1701.robot.subsystems.drive.Drive;
import com.team1701.robot.subsystems.vision.Vision;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
    // Subsystems (Public to avoid use checks)
    public final Drive mDrive = Drive.getInstance();
    public final Vision mVision = Vision.getInstance();

    // Controllers
    private final CommandXboxController mDriverController = new CommandXboxController(0);

    @Override
    public void robotInit() {
        initializeAdvantageKit();
        setupDefaultCommands();
        setupControllerBindings();
    }

    private void initializeAdvantageKit() {
        // Record metadata
        Logger.recordMetadata("RuntimeType", getRuntimeType().toString());
        Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
        Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
        Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
        Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
        Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
        switch (BuildConstants.DIRTY) {
            case 0:
                Logger.recordMetadata("GitDirty", "All changes committed");
                break;
            case 1:
                Logger.recordMetadata("GitDirty", "Uncomitted changes");
                break;
            default:
                Logger.recordMetadata("GitDirty", "Unknown");
                break;
        }

        // Set up data receivers & replay source
        switch (Configuration.getMode()) {
            case REAL:
                Logger.addDataReceiver(new WPILOGWriter("/media/sda1/"));
                Logger.addDataReceiver(new NT4Publisher());
                break;
            case SIMULATION:
                Logger.addDataReceiver(new NT4Publisher());
                break;
            case REPLAY:
                var logPath = LogFileUtil.findReplayLog();
                Logger.setReplaySource(new WPILOGReader(logPath));
                Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
                break;
        }

        // Start AdvantageKit logger
        setUseTiming(Configuration.getMode() != Mode.REPLAY);
        Logger.start();
    }

    private void setupDefaultCommands() {
        mDrive.setDefaultCommand(new JoystickDriveCommand(
                () -> -mDriverController.getLeftY(),
                () -> -mDriverController.getLeftX(),
                () -> -mDriverController.getRightX()));
    }

    private void setupControllerBindings() {
        mDriverController.x().onTrue(Commands.runOnce(() -> mDrive.zeroGyroscope()));
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void autonomousInit() {
        mDrive.zeroModules();
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {
        mDrive.zeroModules();
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void disabledInit() {}

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
