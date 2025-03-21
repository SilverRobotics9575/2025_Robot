// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands;

import frc.robot.Constants.ElevatorConstants;
import frc.robot.OperatorInput;
import frc.robot.subsystems.ElevatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DefaultElevatorCommand extends LoggingCommand {

    private final OperatorInput oi;
    private final ElevatorSubsystem elevatorSubsystem;

    /**
     * Creates a new ElevatorCommand.
     */
    public DefaultElevatorCommand(OperatorInput oi, ElevatorSubsystem elevatorSubsystem) {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(elevatorSubsystem);
        this.oi = oi;
        this.elevatorSubsystem = elevatorSubsystem;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        logCommandStart();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        // Robot is able to go to feeder station & level 1-3
        /* if (oi.feederStation()){
            elevatorSubsystem.level(0);
        }
        else if (oi.level1()) {
            elevatorSubsystem.level(1);
        }
        else if (oi.level2()) {
            elevatorSubsystem.level(2);
        }
        else if (oi.level3()) {
            elevatorSubsystem.level(3);
        }*/
        // Manual control buttons
        if (oi.elevatorUp()) {
            elevatorSubsystem.setElevatorSpeed(ElevatorConstants.CAN_ELEVATOR_MOTOR_SPEED, false, oi.overrideLimit());
        } else if (oi.elevatorDown()) {
            elevatorSubsystem.setElevatorSpeed(-ElevatorConstants.CAN_ELEVATOR_MOTOR_SPEED, true, oi.overrideLimit());
        } else {
            elevatorSubsystem.stop();
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        logCommandEnd(interrupted);
        elevatorSubsystem.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
