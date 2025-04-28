package frc.robot.subsystems.algaeIntake;

import org.littletonrobotics.junction.AutoLog;

public interface AlgaeIntakeIO {
  @AutoLog
  public static class AlgaeIntakeIOInputs {
    /** Some of these may be unnecessary if no NEOs are used. */
    public AlgaeIntakeIOData data = new AlgaeIntakeIOData(
        0.0, 0.0, new double[] {}, new double[] {});
  }

  record AlgaeIntakeIOData(
      double velocityRadsPerSec,
      double appliedVoltage,
      double[] currentAmps,
      double[] tempCelcius) {}

  public default void updateInputs(AlgaeIntakeIOInputs inputs) {}
  /** sets voltage to run motor if necessary */
  public default void setVoltage(double voltage) {}

  /** sets brake mode */
  public default void setBrake(boolean brake) {}
}
