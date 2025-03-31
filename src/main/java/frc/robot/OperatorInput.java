package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutoConstants.AutoPattern;
import frc.robot.Constants.DriveConstants.DriveMode;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.OperatorInputConstants;
import frc.robot.commands.CancelCommand;
import frc.robot.commands.GameController;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.FeederSubsystem;

/**
 * The operatorController exposes all driver functions
 * <p>
 * Extend SubsystemBase in order to have a built in periodic call to support
 * SmartDashboard updates
 */
public class OperatorInput extends SubsystemBase {

    private final GameController operatorController;
    private final GameController driverController;

    // Auto Setup Choosers
    SendableChooser<AutoPattern> autoPatternChooser = new SendableChooser<>();
    SendableChooser<Integer>     waitTimeChooser    = new SendableChooser<>();
    SendableChooser<DriveMode>   driveModeChooser   = new SendableChooser<>();

    /**
     * Construct an OperatorInput class that is fed by a operatorController and
     * optionally an OperatorController.
     */
    public OperatorInput() {

        driverController   = new GameController(OperatorInputConstants.DRIVER_CONTROLLER_PORT,
            OperatorInputConstants.DRIVER_CONTROLLER_DEADBAND);
        operatorController = new GameController(OperatorInputConstants.OPERATOR_CONTROLLER_PORT,
            OperatorInputConstants.DRIVER_CONTROLLER_DEADBAND);

        // Initialize the dashboard selectors
        autoPatternChooser.setDefaultOption("Do Nothing", AutoPattern.DO_NOTHING);
        SmartDashboard.putData("Auto Pattern", autoPatternChooser);
        autoPatternChooser.addOption("Drive Forward", AutoPattern.DRIVE_FORWARD);
        //autoPatternChooser.addOption("Box", AutoPattern.BOX);
        autoPatternChooser.setDefaultOption("Center Level 1", AutoPattern.CENTER_LEVEL1);

        waitTimeChooser.setDefaultOption("No wait", 0);
        SmartDashboard.putData("Auto Wait Time", waitTimeChooser);
        waitTimeChooser.addOption("1 second", 1);
        waitTimeChooser.addOption("3 seconds", 3);
        waitTimeChooser.addOption("5 seconds", 5);

        driveModeChooser.setDefaultOption("Arcade", DriveMode.ARCADE);
        SmartDashboard.putData("Drive Mode", driveModeChooser);
        driveModeChooser.addOption("Tank", DriveMode.TANK);
        driveModeChooser.addOption("Single Stick (L)", DriveMode.SINGLE_STICK_LEFT);
        driveModeChooser.addOption("Single Stick (R)", DriveMode.SINGLE_STICK_RIGHT);
        driveModeChooser.addOption("Slow Mode", DriveMode.SLOW_MODE);
        driveModeChooser.addOption("Single Joystick", DriveMode.SINGLE_JOYSTICK);
    }

    /**
     * Configure the button bindings for all operator commands
     * <p>
     * NOTE: This routine requires all subsystems to be passed in
     * <p>
     * NOTE: This routine must only be called once from the RobotContainer
     *
     * @param driveSubsystem
     */
    public void configureButtonBindings(DriveSubsystem driveSubsystem, ElevatorSubsystem elevatorSubsystem, FeederSubsystem feederSubsystem) {

        // Cancel Command - cancels all running commands on all subsystems
        new Trigger(() -> isCancel())
            .onTrue(new CancelCommand(this, driveSubsystem, elevatorSubsystem, feederSubsystem));

        // Gyro and Encoder Reset
        new Trigger(() -> driverController.getBackButton())
            .onTrue(new InstantCommand(() -> {
                driveSubsystem.resetGyro();
                driveSubsystem.resetEncoders();
            }));
        // Elevator Encoder Reset
        new Trigger(() -> RobotController.getUserButton())
            .onTrue(new InstantCommand(() -> elevatorSubsystem.zeroOnUserButton(true)));
        // Manual control for elevator
        new Trigger(() -> operatorController.getPOV() == 0)
            .whileTrue(new RunCommand(() -> {
                elevatorSubsystem.setElevatorSpeed(ElevatorConstants.CAN_ELEVATOR_MOTOR_SPEED, false, overrideLimit());
            }));
        new Trigger(() -> operatorController.getPOV() == 180)
            .whileTrue(new RunCommand(() -> {
                elevatorSubsystem.setElevatorSpeed(-ElevatorConstants.CAN_ELEVATOR_MOTOR_SPEED, true, overrideLimit());
            }));
    }

    /*
     * Auto Pattern Selectors
     */
    public AutoPattern getAutoPattern() {
        return autoPatternChooser.getSelected();
    }

    public Integer getAutoDelay() {
        return waitTimeChooser.getSelected();
    }

    /*
     * Cancel Command support
     * Do not end the command while the button is pressed
     */
    public boolean isCancel() {
        return operatorController.getStartButton() || driverController.getStartButton();
    }

    /*
     * The following routines are used by the default commands for each subsystem
     *
     * They allow the default commands to get user input to manually move the
     * robot elements.
     */
    /*
     * Drive Subsystem
     */
    public DriveMode getSelectedDriveMode() {
        return driveModeChooser.getSelected();
    }

    /*
     * public boolean isBoost() {
     * // Activates boost mode as long as left axis is held
     * return driverController.getLeftStickButtonPressed();
     * }
     */

    public boolean isSlow() {
        // If the dashboard chooses slow mode then all driving will become slow
        return (driveModeChooser.getSelected() == DriveMode.SLOW_MODE);
    }

    public double getLeftSpeed() {
        if (driveModeChooser.getSelected() == DriveMode.SINGLE_JOYSTICK){
            return operatorController.getLeftY();
        }
        return driverController.getLeftY();

        

    }

    public double getRightSpeed() {
        if (driveModeChooser.getSelected() == DriveMode.SINGLE_JOYSTICK){
            return operatorController.getRightY();
        }
        return driverController.getRightY();
    }

    public double getSpeed() {

        if (driveModeChooser.getSelected() == DriveMode.SINGLE_STICK_RIGHT) {
            return driverController.getRightY();
        }
        if (driveModeChooser.getSelected() == DriveMode.SINGLE_JOYSTICK){
            return operatorController.getLeftY();
        }

        return driverController.getLeftY();
    }

    public double getTurn() {

        if (driveModeChooser.getSelected() == DriveMode.SINGLE_STICK_LEFT) {
            return driverController.getLeftX();
        }
        if (driveModeChooser.getSelected() == DriveMode.SINGLE_JOYSTICK){
            return operatorController.getRightX();
        }
        return driverController.getRightX();
    }

    /*
     * Elevator Subsystem
     */
    // The preset levels for the elevator
    public boolean feederStation(){
        return operatorController.getAButtonPressed();
    }
    public boolean level1() {
        return operatorController.getBButtonPressed();
    }

    public boolean level2() {
        return operatorController.getYButtonPressed();
    }

    public boolean level3() {
        return operatorController.getXButtonPressed();
    }

    public boolean overrideLimit(){
        // When the Y button is held the limit switches will be overrided
        return operatorController.getRightTriggerAxis() > 0.5;
    }
    /*
     * Coral Subsystem
     */

    public boolean scoreCoral() {
        return operatorController.getRightBumperButton();
    }

    public boolean reverseCoral() {
        return operatorController.getLeftBumperButton();
    }

    public boolean stopCoral() {
        return operatorController.getRightBumperButtonReleased() || operatorController.getLeftBumperButtonReleased();
    }

    // * Support for haptic feedback to the driver
    public void startVibrate() {
        driverController.setRumble(GenericHID.RumbleType.kBothRumble, 1);
    }

    public void stopVibrateOperator() {
        operatorController.setRumble(GenericHID.RumbleType.kBothRumble, 0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putString("Driver Controller", operatorController.toString());
    }

}
