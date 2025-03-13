package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.OperatorInput;
import frc.robot.subsystems.FeederSubsystem;

public class DefaultFeederCommand extends LoggingCommand {

    private final OperatorInput   operatorInput;
    private final FeederSubsystem feederSubsystem;

    public DefaultFeederCommand(OperatorInput operatorInput, FeederSubsystem feederSubsystem) {

        this.operatorInput   = operatorInput;
        this.feederSubsystem = feederSubsystem;

        addRequirements(feederSubsystem);
    }

    @Override
    public void execute() {
        if (operatorInput.scoreCoral()) {
            feederSubsystem.setMotorSpeed(-Constants.FeederConstants.FEEDER_MOTOR_INVERTED_SPEED);
        }
        else if (operatorInput.reverseCoral()) {
            feederSubsystem.setMotorSpeed(Constants.FeederConstants.FEEDER_MOTOR_SPEED);
        }
        else if (operatorInput.stopCoral()) {
            feederSubsystem.stop();
        }
    }

    @Override
    public void initialize() {
        logCommandStart();
    }

    @Override
    public boolean isFinished() {
        return false; // default commands never end but can be interrupted
    }

    @Override
    public void end(boolean interrupted) {

        logCommandEnd(interrupted);
    }
}
