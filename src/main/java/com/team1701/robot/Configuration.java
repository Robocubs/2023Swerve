package com.team1701.robot;

import com.team1701.lib.util.LoggedTunableNumber;
import edu.wpi.first.wpilibj.DriverStation;

public final class Configuration {
    private static final RobotType kRobot = RobotType.SWERVE_BOT;
    private static final boolean kTuningEnabled = true;

    static {
        LoggedTunableNumber.enableTuning(kTuningEnabled);
    }

    public static RobotType getRobot() {
        if (Robot.isReal() && kRobot == RobotType.SIMULATION_BOT) {
            DriverStation.reportWarning("Invalid robot configured, using swerve bot as default.", false);
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
