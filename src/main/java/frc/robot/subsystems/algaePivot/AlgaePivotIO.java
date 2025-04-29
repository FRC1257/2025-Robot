package frc.robot.subsystems.algaePivot;

import org.littletonrobotics.junction.AutoLog;

public interface AlgaePivotIO {
  @AutoLog
  public static class AlgaePivotIOInputs {
    public AlgaePivotIOData data =
        new AlgaePivotIOData(0.0, 0.0, 0.0, 0.0, false, new double[] {}, new double[] {});
  }

  record AlgaePivotIOData(
      double angleRads,
      double angVelocityRadsPerSec,
      double appliedVolts,
      double setpointAngleRads,
      boolean breakBeamBroken,
      double[] currentAmps,
      double[] tempCelsius) {}

  /** Updates the set of loggable inputs. */
  public default void updateInputs(AlgaePivotIOInputs inputs) {}

  /** Run open loop at the specified voltage. */
  public default void setVoltage(double motorVolts) {}

  /** Returns the current distance measurement. */
  public default double getAngle() {
    return 0.0;
  }

  /** Returns the angular velocity of the arm in radians per second */
  public default double getAngVelocity() {
    return 0.0;
  }

  public default void setSetpoint(double setpoint) {}

  /** Go to Setpoint */
  public default void goToSetpoint() {}

  public default void setBrake(boolean brake) {}

  public default boolean atSetpoint() {
    return false;
  }

  public default boolean isBreakBeamBroken() {
    return false;
  }

  public default void setP(double p) {}

  public default void setI(double i) {}

  public default void setD(double d) {}

  public default void setkS(double kS) {}

  public default void setkV(double kV) {}

  public default void setkG(double kG) {}

  public default void setkA(double kA) {}

  public default void setActiveP(double p) {}

  public default void setActiveI(double i) {}

  public default void setActiveD(double d) {}

  public default void setActivekS(double kS) {}

  public default void setActivekV(double kV) {}

  public default void setActivekG(double kG) {}

  public default void setActivekA(double kA) {}

  public default double getP() {
    return 0.0;
  }

  public default double getI() {
    return 0.0;
  }

  public default double getD() {
    return 0.0;
  }

  public default double getkS() {
    return 0.0;
  }

  public default double getkG() {
    return 0.0;
  }

  public default double getkV() {
    return 0.0;
  }

  public default double getkA() {
    return 0.0;
  }

  public default double getActiveP() {
    return 0.0;
  }

  public default double getActiveI() {
    return 0.0;
  }

  public default double getActiveD() {
    return 0.0;
  }

  public default double getActivekS() {
    return 0.0;
  }

  public default double getActivekG() {
    return 0.0;
  }

  public default double getActivekV() {
    return 0.0;
  }

  public default double getActivekA() {
    return 0.0;
  }
}
