package frc.robot.util.GenericIOs;

import org.littletonrobotics.junction.AutoLog;

public interface GenericMotorIO {

  @AutoLog
  public static class GenericMotorIOInputs {
    public GenericMotorIOData data = new GenericMotorIOData(0.0, 0.0, 0.0, 0.0, 0.0);
  }

  record GenericMotorIOData(
      double velocityRadPerSec,
      double appliedVolts,
      double torqueCurrentAmps,
      double supplyCurrentAmps,
      double tempCelsius) {}

  public default void updateInputs(GenericMotorIOInputs inputs) {}

  public default void setBrake(boolean brake) {}

  public default void setVoltage(double voltage) {}

  public default void applySetpoint(double setpoint) {}

}
