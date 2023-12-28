package com.team1701.lib.commands;

import java.util.stream.Stream;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class LoggedCommands {
    public static Command logged(Command command) {
        return (command instanceof LoggedCommand) ? command : logged(command.getName(), command);
    }

    public static Command logged(String name, Command command) {
        return new LoggedCommand(name, command);
    }

    public static Command sequence(String name, Command... commands) {
        var command = Commands.sequence(
                Stream.of(commands).map(LoggedCommands::logged).toArray(Command[]::new));
        command.setName(name);
        return command;
    }
}
