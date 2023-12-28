package com.team1701.lib.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WrapperCommand;

public class LoggedCommand extends WrapperCommand {
    private final String mName;
    private final Command mCommand;

    LoggedCommand(String name, Command command) {
        super(command);
        mName = name;
        mCommand = command;
    }

    @Override
    public void initialize() {
        super.initialize();
        CommandLogger.getInstance().onCommandInitialize(mName, mCommand);
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        CommandLogger.getInstance().onCommandEnd(mName, mCommand, interrupted);
    }
}
