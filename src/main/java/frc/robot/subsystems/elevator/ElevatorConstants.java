package frc.robot.subsystems.elevator;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class ElevatorConstants {

  public static final double ELEVATOR_PID_TOLERANCE = Units.degreesToRadians(1);
  public static final double ELEVATOR_PID_VELOCITY_TOLERANCE = 0.5;

  public static final double ELEVATOR_OFFSET = 0; // 1.14;

  public static final double ELEVATOR_PID_TIME = 3;

  public static final double ELEVATOR_MAX_ANGLE = Units.degreesToRadians(110.0);
  public static final double ELEVATOR_MIN_ANGLE = Units.degreesToRadians(2.0);

  public static final double RAMP_RATE = 0.5;
  public static final double STEP_VOLTAGE = 3.0;
  public static final double ELEVATOR_TOLERANCE = 1.0;

  public static int LEFT_MOTOR_ID = 0;
  public static int RIGHT_MOTOR_ID = 1;
  public static IdleMode MOTOR_DEFAULT_IDLE_MODE = IdleMode.kBrake;
  /** Used for converting angular displacement into linear displacement */
  public static double MOTOR_RADIUS_METERS = 1.0;
  /** Gear ratio of the elevator motors */
  public static double GEAR_RATIO = 1.0;

  /** Final position conversion factor based on drum radius and gear ratio */
  public static double POSITION_CONVERSION_FACTOR =
      2 * Constants.PI * ElevatorConstants.MOTOR_RADIUS_METERS * GEAR_RATIO;

  /** 0 radians on the absolute encoder might not match a 0 position height */
  public static double ELEVATOR_OFFSET_METERS = 0.0;
  /** Tolerance used when checking if the elevator is at the setpoint */
  public static double SETPOINT_TOLERANCE_METERS = 5.0;

  public static final double[] kElevatorRealPID = {0, 0, 0, 0};

  public static final double ELEVATOR_MASS_KG = 5;

  public static class ElevatorSimConstants {
    public static final double[] kElevatorSimPID = {15, 0, 0, 0};

    public static final int kMotorPort = 2;
    public static final int kEncoderAChannel = 2;
    public static final int kEncoderBChannel = 3;

    // The P gain for the PID controller that drives this arm.
    public static final double kDefaultArmSetpointDegrees = Units.degreesToRadians(75.0);

    // distance per pulse = (angle per revolution) / (pulses per revolution)
    // = (2 * PI rads) / (4096 pulses)
    public static final double kArmEncoderDistPerPulse = 1 / 4096;

    public static final double kArmReduction = 200;
    public static final double kArmMass = 10.0; // Kilograms
    public static final double kArmLength = Units.inchesToMeters(20);
    public static final double kMinAngleRads = Units.degreesToRadians(0);
    public static final double kMaxAngleRads = Units.degreesToRadians(180);
  }
}
