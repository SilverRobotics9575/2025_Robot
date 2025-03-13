// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose.
 * <p>
 * All constants should be declared globally (i.e. public static).
 * <br>
 * Do not put anything functional in this class.
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final double DEFAULT_COMMAND_TIMEOUT_SECONDS = 5;

    public static final class OperatorInputConstants {

        public static final int    DRIVER_CONTROLLER_PORT     = 1;
        public static final int    OPERATOR_CONTROLLER_PORT   = 0;
        public static final double DRIVER_CONTROLLER_DEADBAND = .2;
    }

    public static final class AutoConstants {

        public static enum AutoPattern {
            DO_NOTHING, DRIVE_FORWARD, BOX, CENTER_LEVEL1;
        }
    }

    public static final class ElevatorConstants {

        public static final int     ELEVATOR_MOTOR_CAN_ID    = 7;
        public static final boolean ELEVATOR_MOTOR_INVERTED  = false;
        public static final double  CAN_ELEVATOR_MOTOR_SPEED = 0.8;

        public static final int     FEEDER_STATION           = 0;
        public static final int     LEVEL1                  = 0;
        public static final int     LEVEL2                  = 0;
        public static final int     LEVEL3                  = 100;

        public static final int MAXHEIGHT_ID = 1;
        public static final int MINHEIGHT_ID = 0;
            
    }

    public static final class FeederConstants {

        public static final int     FEEDER_MOTOR_CAN_ID   = 6;
        public static final boolean FEEDER_MOTOR_INVERTED = true;
        public static final double  FEEDER_MOTOR_SPEED    = 0.6;

    }

    public static final class DriveConstants {

        public static enum DriveMode {
            TANK, ARCADE, SINGLE_STICK_LEFT, SINGLE_STICK_RIGHT, SLOW_MODE, SINGLE_JOYSTICK;
        }

        // NOTE: Follower motors are at CAN_ID+1
        public static final int     LEFT_MOTOR_CAN_ID         = 1;
        public static final int     LEFT_FOLLOW_MOTOR_CAN_ID  = 2;
        public static final int     RIGHT_MOTOR_CAN_ID        = 3;
        public static final int     RIGHT_FOLLOW_MOTOR_CAN_ID = 4;

        public static final boolean LEFT_MOTOR_INVERTED       = false;
        public static final boolean RIGHT_MOTOR_INVERTED      = true;

        public static final double  CM_PER_ENCODER_COUNT      = 3.503;

        public static final boolean GYRO_INVERTED             = false;

        /**
         * Proportional gain for gyro pid tracking
         */
        public static final double  GYRO_PID_KP               = 0.01;

        public static final double  DRIVE_SCALING_BOOST       = 0.5;
        public static final double  DRIVE_SCALING_NORMAL      = .2;
        public static final double  DRIVE_SCALING_SLOW        = .1;
        public static final double SLEW_RATE_LIMIT = 0.5;
    }

    public static final class LightsConstants {

        public static final int LED_STRING_PWM_PORT = 0;
        public static final int LED_STRING_LENGTH   = 60;
    }
}
