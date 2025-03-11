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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;


/* 
    the proportional term drives the position error to zero, 
    the derivative term drives the velocity error to zero
    integral term drives the total accumulated error-over-time to zero.
 */
public class ElevatorSubsystem extends SubsystemBase {

    // private final LightsSubsystem lightsSubsystem;
    private final SparkMax            elevatorMotor                = new SparkMax(ElevatorConstants.ELEVATOR_MOTOR_CAN_ID,
        MotorType.kBrushless);

    private final RelativeEncoder     elevatorEncoder              = elevatorMotor.getEncoder();

    private SparkClosedLoopController elevatorClosedLoopController = elevatorMotor.getClosedLoopController();

    private double                    elevatorEncoderOffset        = 0;
    private double                    elevatorSpeed                = 0;
    private double                    elevatorCurrentTarget        = ElevatorConstants.kFeederStation;
    private final DigitalInput        maxHeight                    = new DigitalInput(ElevatorConstants.MAXHEIGHT_ID);
    private final DigitalInput        minHeight                    = new DigitalInput(ElevatorConstants.MINHEIGHT_ID);

    public enum ElevatorPosition {
    }


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
    public void level(int level) {
        switch (level) {

        /*case kFeederStation ->
            elevatorCurrentTarget = ElevatorConstants.kFeederStation; */
        case 1 ->
            elevatorCurrentTarget = ElevatorConstants.kLevel1;
        case 2 ->
            elevatorCurrentTarget = ElevatorConstants.kLevel2;
        case 3 ->
            elevatorCurrentTarget = ElevatorConstants.kLevel3;
        }
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

    @Override
    public void periodic() {
        moveToSetpoint();
        // zeroOnUserButton();
        // This method will be called once per scheduler run
        // Display the position and target position of the elevator on the SmartDashboard
        SmartDashboard.putNumber("Elevator Target Position", Math.round(elevatorCurrentTarget * 100) / 100d);
        SmartDashboard.putNumber("Elevator Actual Position", Math.round(elevatorEncoder.getPosition()) *100 / 100d);
    }


    public void setElevatorSpeed(double motorSpeed, boolean down) {
        elevatorSpeed = motorSpeed;
        
        // Program for the limit switches
        
         System.out.println(minHeight.get());
         if (minHeight.get() && down) {
            System.out.println("WARNING: Minimum height reached");
            SmartDashboard.putString("Limit Switch Status", "WARNING: MIN HEIGHT");
            }
         if (maxHeight.get() && !down){
            System.out.println("WARNING: Maximum height reached");
            SmartDashboard.putString("Limit Switch Status", "WARNING: MAX HEIGHT");
            }
         else {
            elevatorMotor.set(elevatorSpeed);
            SmartDashboard.putString("Limit Switch Status", "Ok");
         }
         
        elevatorMotor.set(elevatorSpeed);
    }

    public void stop() {
        elevatorMotor.set(0);
    }
}
