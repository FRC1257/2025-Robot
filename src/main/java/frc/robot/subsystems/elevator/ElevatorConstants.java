package frc.robot.subsystems.elevator;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class ElevatorConstants {
    /** current limit on the elevator motor in amps */
    public static int MOTOR_CURRENT_LIMIT = 50;
    public static int LEFT_MOTOR_ID = 0;
    public static int RIGHT_MOTOR_ID = 1;
    public static IdleMode MOTOR_DEFAULT_IDLE_MODE = IdleMode.kBrake;
    /** Used for converting angular displacement into linear displacement */
    public static double MOTOR_RADIUS_METERS = 1.0;
    /** 0 radians on the absolute encoder might not match a 0 position height */
    public static double ABSOLUTE_ENCODER_OFFSET_RADIANS = 0.0;
    /** Tolerance used when checking if the elevator is at the setpoint */
    public static double SETPOINT_TOLERANCE_METERS = 5.0;
    public static double kP;
    public static double kI;
    public static double kD;
    public static double KF;
}
