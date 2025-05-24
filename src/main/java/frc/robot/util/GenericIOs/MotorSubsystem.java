package frc.robot.util.GenericIOs;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

public class MotorSubsystem<SubsystemIO extends GenericMotorIO> extends SubsystemBase {

  protected final SubsystemIO io;
  protected final GenericMotorIOInputsAutoLogged inputs = new GenericMotorIOInputsAutoLogged();

  public MotorSubsystem(SubsystemIO io) {
    super();
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(this.getName(), inputs);
  }

  public void setBrake(boolean brake) {
    io.setBrake(brake);
  }

  public Command setpointCommand(double setpoint) {
    return runOnce(() -> io.applySetpoint(setpoint));
  }

  public Command followSetpointCommand(DoubleSupplier setpoint) {
    return run(() -> io.applySetpoint(setpoint.getAsDouble()));
  }
}
