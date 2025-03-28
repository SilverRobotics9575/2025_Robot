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

import edu.wpi.first.wpilibj.DigitalInput;
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

    // private final LightsSubsystem lightsSubsystem;
    private final SparkMax elevatorMotor = new SparkMax(ElevatorConstants.ELEVATOR_MOTOR_CAN_ID, MotorType.kBrushless);
    private final RelativeEncoder elevatorEncoder = elevatorMotor.getEncoder();
    private final SparkClosedLoopController elevatorClosedLoopController = elevatorMotor.getClosedLoopController();

    private double elevatorEncoderOffset = 0;
    private double elevatorSpeed = 0;
    private double elevatorCurrentTarget = ElevatorConstants.LEVEL1; // The starting position should be at level 1

    private final DigitalInput maxHeight = new DigitalInput(ElevatorConstants.MAXHEIGHT_ID);
    private final DigitalInput minHeight = new DigitalInput(ElevatorConstants.MINHEIGHT_ID);

    private boolean wasResetByButton = false;
    private boolean wasResetByLimit = false;

    public ElevatorSubsystem(LightsSubsystem lightsSubsystem) {

        // Configure the elevator motor
        SparkMaxConfig config = new SparkMaxConfig();
        config.inverted(ElevatorConstants.ELEVATOR_MOTOR_INVERTED)
                .idleMode(IdleMode.kBrake)
                .disableFollowerMode();
        elevatorMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        elevatorEncoder.setPosition(0);
    }

    public enum Setpoint {
        FEEDER_STATION, LEVEL1, LEVEL2, LEVEL3;
    }

    /**
     * Resets the elevator encoder to zero.
     */
    /*public void resetEncoders() {
        // Reset the offsets so that the encoders are zeroed.
        elevatorEncoderOffset  = 0;
        elevatorEncoderOffset  = -getElevatorEncoder();
    }

    public double getElevatorEncoder() {
        return elevatorEncoder.getPosition() + elevatorEncoderOffset;
    }*/

    /**
     * Drive the arm and elevator motors to their respective setpoints. This
     * will use MAXMotion position control which will allow for a smooth
     * acceleration and deceleration to the mechanisms' setpoints.
     */
    private void moveToSetpoint() {
        elevatorClosedLoopController.setReference(
                elevatorCurrentTarget, ControlType.kPosition);
    }

    // Zero the elevator encoder when the limit switch or a button is pressed
    private void zeroElevatorOnLimit() {
        if (minHeight.get() && !wasResetByLimit) {
            // Zero the encoder only when the limit switch is pressed
            // to prevent constant zeroing while pressed
            elevatorEncoder.setPosition(0);
            wasResetByLimit = true;
        } else if (!minHeight.get()) {
            wasResetByLimit = false;
        }
    }

    // Zero the encoder when the robo rio user button is
    public void zeroOnUserButton(boolean resetEncoders) {
        if (resetEncoders && !wasResetByButton) {
            // Zero the encoders only when the button switches from pressed 
            // to prevent constant zeroing while pressed
            wasResetByButton = true;
            elevatorEncoder.setPosition(0);
        } else if (!resetEncoders) {
            wasResetByButton = false;
        }
    }

    /**
     * Command to set the subsystem setpoint. This will set the elevator to
     * their predefined positions for the given setpoint.
     */
    public Command setSetpointCommand(Setpoint setpoint) {
        return this.runOnce(
                () -> {
                    switch (setpoint) {
                        case FEEDER_STATION ->
                            elevatorCurrentTarget = ElevatorConstants.FEEDER_STATION;
                        case LEVEL1 ->
                            elevatorCurrentTarget = ElevatorConstants.LEVEL1;
                        case LEVEL2 ->
                            elevatorCurrentTarget = ElevatorConstants.LEVEL2;
                        case LEVEL3 ->
                            elevatorCurrentTarget = ElevatorConstants.LEVEL3;
                    }
                });
    }

    @Override
    public void periodic() {
        moveToSetpoint();
        //zeroElevatorOnLimit();
        // This method will be called once per scheduler run
        // Display the position and target position of the elevator on the SmartDashboard
        SmartDashboard.putNumber("Elevator Target Position", Math.round(elevatorCurrentTarget * 100) / 100d);
        SmartDashboard.putNumber("Elevator Actual Position", Math.round(elevatorEncoder.getPosition()) * 100 / 100d);
    }

    // The manual control method
    public void setElevatorSpeed(double motorSpeed, boolean down, boolean isOverridePressed) {
        String limString = "Limit Switch Status";
        elevatorSpeed = motorSpeed;

        if (isOverridePressed) {
            // Override limit switches
            elevatorMotor.set(elevatorSpeed);
            SmartDashboard.putString(limString, "Override");
            System.out.println("WARNING: Elevator limit overrided");
        } // Normal operation with limit switches
        else {
            // when minimum height is reached and controller is attempting to go down. STOP
            if (minHeight.get() && down) {
                System.out.println("WARNING: Minimum height reached");
                SmartDashboard.putString(limString, "WARNING: MIN HEIGHT");
                //setElevatorHigher();
            } // When maximum height is reached and elevator is attempting to go up. STOP 
            else if (maxHeight.get() && !down) {
                System.out.println("WARNING: Maximum height reached");
                SmartDashboard.putString(limString, "WARNING: MAX HEIGHT");
            } // If all is good let the elevator run
            else {
                elevatorMotor.set(elevatorSpeed);
                SmartDashboard.putString(limString, "Ok");
            }
        }
    }

    // Slightly sets the elevator higher after the bottom limit switch is triggered
    // TODO: Test and incorporate to main
   /* private void setElevatorHigher() {
        if (minHeight.get()) {
            elevatorMotor.set(ElevatorConstants.ELEVATOR_REBOUND_SPEED);
            Timer.delay(ElevatorConstants.ELEVATOR_REBOUND_TIME);
            elevatorMotor.set(0);
        }
    }*/

    public void stop() {
        elevatorMotor.set(0);
    }
}
