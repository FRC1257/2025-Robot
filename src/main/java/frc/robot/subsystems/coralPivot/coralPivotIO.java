package frc.robot.subsystems.coralPivot;

import org.littletonrobotics.junction.AutoLog;

public interface coralPivotIO {
  @AutoLog
  public static class coralPivotIOInputs {
    public double angleRads = 0.0;
    public double angVelocityRadsPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double setpointAngleRads = 0.0;
    public double[] currentAmps = new double[] {};
    public double[] tempCelsius = new double[] {};
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(coralPivotIOInputs inputs) {}

  /** Run open loop at the specified voltage. */
  public default void setVoltage(double motorVolts) {}

  /** Returns the current distance measurement. */
  public default double getAngle() {
    return 0.0;
  }

  /** Sets the pivot arm voltage to 0 */
  public default void stop() {}

  /** Go to Setpoint */
  public default void goToSetpoint(double setpoint) {}

  public default void setBrake(boolean brake) {}

  public default boolean atSetpoint() {
    return false;
  }

  public default void setP(double p) {}

  public default void setI(double i) {}

  public default void setD(double d) {}

  public default double getP() {
    return 0.0;
  }

  public default double getI() {
    return 0.0;
  }

  public default double getD() {
    return 0.0;
  }
}
