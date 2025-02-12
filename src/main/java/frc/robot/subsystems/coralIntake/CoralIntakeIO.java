package frc.robot.subsystems.coralIntake;

import org.littletonrobotics.junction.AutoLog;

public interface CoralIntakeIO {
  @AutoLog
  public static class CoralIntakeIOInputs {
    /** Some of these may be unnecessary if no NEOs are used. */
    public double velocityRadsPerSec = 0.0;

    public double appliedVoltage = 0.0;
    public double[] currentAmps = new double[] {};
    public double[] tempCelcius = new double[] {};
  }

  public default void setBrake(boolean brake) {}

  public default void updateInputs(CoralIntakeIOInputs inputs) {}
  /** sets voltage to run motor if necessary */
  public default void setVoltage(double voltage) {}
  /** sets velocity setpoint */
}

  /** sets brake mode */
