package frc.robot.subsystems.elevator;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import frc.robot.Constants;

public class ElevatorConstants {
  // top height 50 inches (measured from bottom of elevator frame)
  // bottom height 12.375 inches (measured from bottom of frame)
  public static final double ELEVATOR_MAX_HEIGHT = 0.955675; // not including chasis
  public static final double ELEVATOR_MIN_HEIGHT = 0;

  // sysid stuff
  public static final double RAMP_RATE = 0.5;
  public static final double STEP_VOLTAGE = 3.0;
  public static final double ELEVATOR_TOLERANCE = 1.0;

  public static int LEFT_MOTOR_ID = 10;
  public static int RIGHT_MOTOR_ID = 5;
  public static IdleMode MOTOR_DEFAULT_IDLE_MODE = IdleMode.kBrake;
  /** Used for converting angular displacement into linear displacement */
  public static double DRUM_RADIUS_METERS = 0.02;
  /** Gear ratio of the elevator motors */
  public static double GEAR_RATIO = 1.0 / 20.0;

  /** Final position conversion factor based on drum radius and gear ratio */
  public static double POSITION_CONVERSION_FACTOR =
      2 * Constants.PI * DRUM_RADIUS_METERS * GEAR_RATIO;

  /** When the elevator is on the bottom, what does the encoder say */
  public static double ELEVATOR_OFFSET_METERS = 0.0;

  public static final double ELEVATOR_MASS_KG = 5;

  // Max velocity and acceleration of the elevator, in m/s and m/s^2
  public static final double MAX_VELOCITY = 2;
  public static final double MAX_ACCELERATION = 2;

  // Digital input channels
  public static final int ABSOLUTE_ENCODER_CHANNEL = 1;
  public static final int LIMIT_SWITCH_CHANNEL = 0;

  // Height setpoints for elevator
  public static final double ELEVATOR_INTAKE_HEIGHT = 0.3;
  public static final double ELEVATOR_L1_HEIGHT = 0.4;
  public static final double ELEVATOR_L2_HEIGHT = 0.6;
  public static final double ELEVATOR_L3_HEIGHT = 0.7;

  // PID Constants
  public static final double[] ELEVATOR_REAL_PID = {0.1, 0, 0, 0};
  /** Tolerance used when checking if the elevator is at the setpoint */
  public static double SETPOINT_TOLERANCE_METERS = 0.01;

  public static class ElevatorSimConstants {
    public static final double[] ELEVATOR_SIM_PID = {15, 0, 0, 2.261};
    // Convert from encoder steps to meters

    public static final double GEAR_RATIO_SIM = 1;
    public static final double ELEVATOR_STARTING_HEIGHT = 0.2;

    // 4096 pulses per revolution
    // (2pi radians / 4096) * gear ratio
    public static final double ENCODER_DIST_PER_PULSE =
        2 * Math.PI / 4096 * DRUM_RADIUS_METERS * GEAR_RATIO;
    // public static final int kMotorPort = 2;

  }
}
