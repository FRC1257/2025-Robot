package frc.robot.subsystems.algaePivot;

import edu.wpi.first.math.util.Units;

public class AlgaePivotConstants {
  // May have to change these constants, same as pivot arm from last year right now

  public static final int ALGAE_PIVOT_ID = 0; // Change later

  public static final double POSITION_CONVERSION_FACTOR = 21.0 / 35.0;
  public static final double ALGAE_PIVOT_ROTATION_DIAM_M = 1;

  public static final double[] ALGAE_PIVOT_PID_REAL = {3.6, 0, 0, 0.01};
  public static final double[] ALGAE_PIVOT_FEEDFORWARD_REAL = {0, 0.45, 0, 0};

  public static final double ALGAE_PIVOT_PID_TOLERANCE = Units.degreesToRadians(1);
  public static final double ALGAE_PIVOT_PID_VELOCITY_TOLERANCE = 0.5;

  public static final double ALGAE_PIVOT_OFFSET = 1.5; // 1.14;

  public static final double PIVOT_MAX_PID_TIME = 3;

  public static final double ALGAE_PIVOT_MAX_ANGLE = Units.degreesToRadians(100);
  public static final double ALGAE_PIVOT_MIN_ANGLE = Units.degreesToRadians(-20);

  // deleted the old constants from last years code, this intake angle is from last year still
  public static final double ALGAE_PIVOT_DOWN_ANGLE = Units.degreesToRadians(20);
  public static final double ALGAE_PIVOT_STOW_ANGLE = Units.degreesToRadians(95);
  public static final double ALGAE_PIVOT_AUTO_INTAKE_ANGLE = Units.degreesToRadians(45);
  public static final double ALGAE_PIVOT_PROCESSOR_ANGLE = Units.degreesToRadians(45);
  // Will have to add constants for placing ALGAE

  public static final double RAMP_RATE = 0.5;
  public static final double STEP_VOLTAGE = 3.0;
  public static final double ALGAE_PIVOT_TOLERANCE = 1.0;

  public static final double ALGAE_PIVOT_CONTROL_SPEED_FACTOR = 1.0;

  public static final double ALGAE_PIVOT_MAX_VELOCITY = 0.3;
  public static final double ALGAE_PIVOT_MAX_ACCELERATION = 0.3;

  public static final int BREAK_BEAM_CHANNEL = 0;

  public static class AlgaePivotSimConstants {
    public static final double[] kPivotSimPID = {15, 0, 0, 0};
    public static final double[] kPivotSimFF = {0, 0.574, 0, 0};

    // The P gain for the PID controller that drives this arm.
    public static final double kDefaultArmSetpointDegrees = Units.degreesToRadians(75.0);

    // distance per pulse = (angle per revolution) / (pulses per revolution)
    // = (2 * PI rads) / (4096 pulses)
    public static final double kArmEncoderDistPerPulse = 1 / 4096;

    public static final double kArmReduction = 200;
    public static final double kArmMass = 10.0; // Kilograms
    public static final double kArmLength = Units.inchesToMeters(20);
  }
}
