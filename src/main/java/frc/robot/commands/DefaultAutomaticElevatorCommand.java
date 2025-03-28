// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands;

import frc.robot.Constants.ElevatorConstants;
import frc.robot.OperatorInput;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.Setpoint;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DefaultAutomaticElevatorCommand extends LoggingCommand {

    private final OperatorInput     oi;
    private final ElevatorSubsystem elevatorSubsystem;

    /**
     * Creates a new ElevatorCommand.
     */
    public DefaultAutomaticElevatorCommand(OperatorInput oi, ElevatorSubsystem elevatorSubsystem) {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(elevatorSubsystem);
        this.oi                = oi;
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
        if (oi.feederStation()){
            elevatorSubsystem.setSetpointCommand(Setpoint.FEEDER_STATION);
        }
        else if (oi.level1()) {
            elevatorSubsystem.setSetpointCommand(Setpoint.LEVEL1);
        }
        else if (oi.level2()) {
            elevatorSubsystem.setSetpointCommand(Setpoint.LEVEL2);
        }
        else if (oi.level3()) {
            elevatorSubsystem.setSetpointCommand(Setpoint.LEVEL3);
        }
        
        // TODO: If auto works then move the manual control to the manual elevator command
        // TODO: Test that the speeds accurately work
        // Manual control buttons
        if (oi.elevatorUp() > 0) {
            elevatorSubsystem.setElevatorSpeed(Math.abs(oi.elevatorUp())* ElevatorConstants.CAN_ELEVATOR_MOTOR_SPEED, false, oi.overrideLimit());
        }
        else if (oi.elevatorDown() > 0) {
            elevatorSubsystem.setElevatorSpeed(- Math.abs(oi.elevatorDown()) * ElevatorConstants.CAN_ELEVATOR_MOTOR_SPEED, true, oi.overrideLimit());
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
