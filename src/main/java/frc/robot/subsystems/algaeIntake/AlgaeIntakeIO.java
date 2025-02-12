package frc.robot.subsystems.algaeIntake;

import org.littletonrobotics.junction.AutoLog;

public interface AlgaeIntakeIO {
  @AutoLog
  public static class AlgaeIntakeIOInputs {
    /** Some of these may be unnecessary if no NEOs are used. */
    public double velocityRadsPerSec = 0.0;

    public double appliedVoltage = 0.0;
    public double[] currentAmps = new double[] {};
    public double[] tempCelcius = new double[] {};
  }

  public default void updateInputs(AlgaeIntakeIOInputs inputs) {}
  /** sets voltage to run motor if necessary */
  public default void setVoltage(double voltage) {}

  /** sets brake mode */
  public default void setBrake(boolean brake) {}
}
