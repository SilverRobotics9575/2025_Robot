// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;


/* 
    the proportional term drives the position error to zero, 
    the derivative term drives the velocity error to zero
    integral term drives the total accumulated error-over-time to zero.
 */
public class ElevatorSubsystem extends SubsystemBase {

    public enum Setpoint {
        K_FEEDERSTATION,
        KLEVEL1,
        KLEVEL2,
        KLEVEL3,
        KLEVEL4;
    }

    // private final LightsSubsystem lightsSubsystem;
    private final SparkMax elevatorMotor
            = new SparkMax(ElevatorConstants.ELEVATOR_MOTOR_CAN_ID, MotorType.kBrushless);

    private final RelativeEncoder elevatorEncoder
            = elevatorMotor.getEncoder();

    private SparkClosedLoopController elevatorClosedLoopController
            = elevatorMotor.getClosedLoopController();

    private double elevatorEncoderOffset = 0;
    private double elevatorSpeed = 0;
    private double elevatorCurrentTarget = ElevatorConstants.kFeederStation;

    /**
     * Creates a new ElevatorSubsystem.
     */
    public ElevatorSubsystem(LightsSubsystem lightsSubsystem) {

        // Configure the elevator motor
        SparkMaxConfig config = new SparkMaxConfig();
        config.inverted(ElevatorConstants.ELEVATOR_MOTOR_INVERTED)
                .idleMode(IdleMode.kBrake)
                .disableFollowerMode();
        elevatorMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    /**
     * Command to set the subsystem setpoint. This will set the arm and elevator
     * to their predefined positions for the given setpoint.
     */
    public Command setSetpointCommand(Setpoint setpoint) {
        return this.runOnce(
                () -> {
                    switch (setpoint) {
                        case K_FEEDERSTATION ->
                            elevatorCurrentTarget = ElevatorConstants.kFeederStation;
                        case KLEVEL1 ->
                            elevatorCurrentTarget = ElevatorConstants.kLevel1;
                        case KLEVEL2 ->
                            elevatorCurrentTarget = ElevatorConstants.kLevel2;
                        case KLEVEL3 ->
                            elevatorCurrentTarget = ElevatorConstants.kLevel3;
                        case KLEVEL4 ->
                            elevatorCurrentTarget = ElevatorConstants.kLevel4;
                    }
                });
    }

    /**
     * Drive the arm and elevator motors to their respective setpoints. This
     * will use MAXMotion position control which will allow for a smooth
     * acceleration and deceleration to the mechanisms' setpoints.
     */
    private void moveToSetpoint() {
        elevatorClosedLoopController.setReference(
                elevatorCurrentTarget, ControlType.kMAXMotionPositionControl);
    }

// Should consider adding a zeroOnUserButton method to zero the encoders when the user button is pressed
    /*
    private void zeroOnUserButton() {
        if (!wasResetByButton && RobotController.getUserButton()) {
            // Zero the encoders only when button switches from "unpressed" to "pressed" to prevent
            // constant zeroing while pressed
            wasResetByButton = true;
            armEncoder.setPosition(0);
            elevatorEncoder.setPosition(0);
        } else if (!RobotController.getUserButton()) {
            wasResetByButton = false;
        }
    }*/
    @Override
    public void periodic() {
        moveToSetpoint();
        // zeroOnUserButton();
        // This method will be called once per scheduler run

        // Display the position and target position of the elevator on the SmartDashboard
        SmartDashboard.putNumber("Elevator Target Position", elevatorCurrentTarget);
        SmartDashboard.putNumber("Elevator Actual Position", elevatorEncoder.getPosition());
    }

    public void stop() {
        elevatorMotor.set(0);
    }
}
