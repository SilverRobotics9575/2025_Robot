package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.OperatorInput;
import frc.robot.subsystems.FeederSubsystem;

public class FeederCommand extends LoggingCommand {

    private final OperatorInput   operatorInput;
    private final FeederSubsystem feederSubsystem;

    public FeederCommand(OperatorInput operatorInput, FeederSubsystem feederSubsystem) {

        this.operatorInput   = operatorInput;
        this.feederSubsystem = feederSubsystem;

        addRequirements(feederSubsystem);
    }

    @Override
    public void execute() {
        if (operatorInput.feeder()) {
            feederSubsystem.setMotorSpeed(Constants.FeederConstants.FEEDER_MOTOR_SPEED);
        }
        if (operatorInput.feederstop()) {
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
