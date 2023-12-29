package com.team1701.robot;

import com.team1701.lib.alerts.Alert;
import com.team1701.lib.util.LoggedTunableNumber;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public final class Configuration {
    private static final RobotType kRobot = RobotType.SIMULATION_BOT;
    private static final boolean kTuningEnabled = true;

    static {
        LoggedTunableNumber.enableTuning(kTuningEnabled);
    }

    public static RobotType getRobot() {
        if (Robot.isReal() && kRobot == RobotType.SIMULATION_BOT) {
            Alert.warning(Configuration.class, "Invalid robot configured. Using swerve bot as default.");
            return RobotType.SWERVE_BOT;
        }

        return kRobot;
    }

    public static Mode getMode() {
        switch (getRobot()) {
            case SWERVE_BOT:
                return Robot.isReal() ? Mode.REAL : Mode.REPLAY;
            case SIMULATION_BOT:
                return Mode.SIMULATION;
            default:
                return Mode.REAL;
        }
    }

    public static Alliance getAlliance() {
        return DriverStation.getAlliance().orElse(Alliance.Blue);
    }

    public static enum RobotType {
        SWERVE_BOT,
        SIMULATION_BOT
    }

    public static enum Mode {
        REAL,
        REPLAY,
        SIMULATION
    }
}
