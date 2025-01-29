package frc.robot.subsystems.elevator;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants;

public class ElevatorConstants {
    /** current limit on the elevator motor in amps */
    public static int MOTOR_CURRENT_LIMIT = 50;
    public static int LEFT_MOTOR_ID = 0;
    public static int RIGHT_MOTOR_ID = 1;
    public static IdleMode MOTOR_DEFAULT_IDLE_MODE = IdleMode.kBrake;
    /** Used for converting angular displacement into linear displacement */
    public static double MOTOR_RADIUS_METERS = 1.0;
    /** Gear ratio of the elevator motors */
    public static double GEAR_RATIO = 1.0;

    /** Final position conversion factor based on drum radius and gear ratio */
    public static double POSITION_CONVERSION_FACTOR = 2 * Constants.PI * ElevatorConstants.MOTOR_RADIUS_METERS * GEAR_RATIO;

    /** 0 radians on the absolute encoder might not match a 0 position height */
    public static double ELEVATOR_OFFSET_METERS = 0.0;
    /** Tolerance used when checking if the elevator is at the setpoint */
    public static double SETPOINT_TOLERANCE_METERS = 5.0;
    public static double kP = 0.0;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kFF = 0.0;
}
