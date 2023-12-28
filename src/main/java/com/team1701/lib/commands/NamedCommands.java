package com.team1701.lib.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class NamedCommands {
    public static Command named(String name, Command command) {
        command.setName(name);
        return command;
    }

    public static Command runOnce(String name, Runnable action, Subsystem... requirements) {
        var command = Commands.runOnce(action, requirements);
        command.setName(name);
        return command;
    }
}
