package frc.robot.subsystems.algaePivot;

import edu.wpi.first.math.util.Units;

public class AlgaePivotConstants {
  // May have to change these constants, same as pivot arm from last year right now

  public static final int CORAL_PIVOT_ID = 0; // Change later

  public static final double POSITION_CONVERSION_FACTOR = 21.0 / 35.0;
  public static final double CORAL_PIVOT_ROTATION_DIAM_M = 1;

  public static final double[] CORAL_PIVOT_PID_REAL = {3.6, 0, 0, 0.01};
  public static final double[] CORAL_PIVOT_FEEDFORWARD_REAL = {0, 0.45, 0, 0};

  public static final double CORAL_PIVOT_PID_TOLERANCE = Units.degreesToRadians(1);
  public static final double CORAL_PIVOT_PID_VELOCITY_TOLERANCE = 0.5;

  public static final double CORAL_PIVOT_OFFSET = 1.5; // 1.14;

  public static final double PIVOT_MAX_PID_TIME = 3;

  public static final double CORAL_PIVOT_MAX_ANGLE = Units.degreesToRadians(90);
  public static final double CORAL_PIVOT_MIN_ANGLE = Units.degreesToRadians(-70);

  // deleted the old constants from last years code, this intake angle is from last year still
  public static final double CORAL_PIVOT_INTAKE_ANGLE = Units.degreesToRadians(2.0);
  // Will have to add constants for placing algae

  public static final double RAMP_RATE = 0.5;
  public static final double STEP_VOLTAGE = 3.0;
  public static final double CORAL_PIVOT_TOLERANCE = 1.0;

  public static final double CORAL_PIVOT_CONTROL_SPEED_FACTOR = 1.0;

  public static final double CORAL_PIVOT_MAX_VELOCITY = 0.3;
  public static final double CORAL_PIVOT_MAX_ACCELERATION = 0.3;

  public static class CoralPivotSimConstants {
    public static final double[] kPivotSimPID = {15, 0, 0, 0};
    public static final double[] kPivotSimFF = {0, 0.574, 0, 0};

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
    public static final double kMinAngleRads = Units.degreesToRadians(-70);
    public static final double kMaxAngleRads = Units.degreesToRadians(90);
  }
}