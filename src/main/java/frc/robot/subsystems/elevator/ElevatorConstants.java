package frc.robot.subsystems.elevator;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class ElevatorConstants {

  public static final double ELEVATOR_PID_TOLERANCE = Units.degreesToRadians(1);
  public static final double ELEVATOR_PID_VELOCITY_TOLERANCE = 0.5;

  public static final double ELEVATOR_OFFSET = 0; // 1.14;

  //top height 50 inches (measured from bottom of elevator frame)
  //bottom height 12.375 inches (measured from bottom of frame)
  public static final double ELEVATOR_MAX_HEIGHT = 0.955675; //not including chasis
  public static final double ELEVATOR_MIN_HEIGHT = 0;

  //sysid stuff
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

  /** When the elevator is on the bottom, what does the encoder say */
  public static double ELEVATOR_OFFSET_METERS = 0.0;
  /** Tolerance used when checking if the elevator is at the setpoint */
  public static double SETPOINT_TOLERANCE_METERS = 0.01;

  public static final double[] kElevatorRealPID = {0, 0, 0, 0};

  public static final double ELEVATOR_MASS_KG = 5;

  public static class ElevatorSimConstants {
    public static final double[] kElevatorSimPID = {15, 0, 0, 0};

    // public static final int kMotorPort = 2;
    // public static final int kEncoderAChannel = 2;
    // public static final int kEncoderBChannel = 3;

  }
}
