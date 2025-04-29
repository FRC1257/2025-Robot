package frc.robot.subsystems.coralIntake;

import org.littletonrobotics.junction.AutoLog;

public interface CoralIntakeIO {
  @AutoLog
  public static class CoralIntakeIOInputs {

    public CoralIntakeIOData data =
        new CoralIntakeIOData(0.0, 0.0, new double[] {}, new double[] {}, false);
  }

  record CoralIntakeIOData(
      double velocityRadsPerSec,
      double appliedVoltage,
      double[] currentAmps,
      double[] tempCelcius,
      boolean isBreakBeamBroken) {}

  public default void setBrake(boolean brake) {}

  public default void updateInputs(CoralIntakeIOInputs inputs) {}
  /** sets voltage to run motor if necessary */
  public default void setVoltage(double voltage) {}

  public default boolean isBreakBeamBroken() {
    return false;
  }

  public default double getVelocity() {
    return 0;
  }
}

  /** sets brake mode */
