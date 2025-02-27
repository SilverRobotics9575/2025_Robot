package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class FeederSubsystem extends SubsystemBase {

    private double MotorSpeed = 0;
    private final SparkMax feederMotor = new SparkMax(Constants.FeederConstants.FEEDER_MOTOR_CAN_ID, MotorType.kBrushless);

    // For now, this is a basic, bare-bones subsystem, since I need it to work by Saturday.
    public FeederSubsystem(LightsSubsystem lightsSubsystem) {

        SparkMaxConfig config = new SparkMaxConfig();
        config.inverted(true)
                .idleMode(IdleMode.kBrake)
                .disableFollowerMode();
        feederMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setMotorSpeed(double motorSpeed) {
        this.MotorSpeed = motorSpeed;

        feederMotor.set(this.MotorSpeed);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Feeder Motor", MotorSpeed);
    }

    public void stop() {
        feederMotor.set(0);
    }

}
