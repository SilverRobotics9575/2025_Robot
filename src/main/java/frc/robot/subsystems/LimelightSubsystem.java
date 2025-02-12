package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class LimelightSubsystem extends SubsystemBase {

    public LimelightSubsystem() {
        // Initialization code goes here
    }

    // Returns true if a target is detected (AprilTag).
    public boolean hasTarget() {
        return LimelightHelpers.getTV("");
    }

    // Get horizontal offset from crosshair in degrees.
    public double getXOffset() {
        return LimelightHelpers.getTX("");
    }

    // Get vertical offset from crosshair in degrees.
    public double getYOffset() {
        return LimelightHelpers.getTY("");
    }

    // Get the target area, percentage of the image area
    public double getTargetArea() {
        return LimelightHelpers.getTA("");
    }
}
