// NOTE: This file is experimental, careful testing with the robot.

package frc.robot.commands.drive;

import frc.robot.commands.LoggingCommand;
import frc.robot.subsystems.DriveSubsystem;

public class TurnOnHeadingCommand extends LoggingCommand {

    private final double         angle, speed, timeoutSeconds;
    private final boolean        brakeAtEnd;
    private final DriveSubsystem driveSubsystem;
}
