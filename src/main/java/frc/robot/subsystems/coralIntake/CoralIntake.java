package frc.robot.subsystems.coralIntake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class CoralIntake extends SubsystemBase {
  private final CoralIntakeIO io;
  CoralIntakeIOInputsAutoLogged inputs = new CoralIntakeIOInputsAutoLogged();

  public CoralIntake(CoralIntakeIO io) {
    this.io = io;
    SmartDashboard.putData(getName(), this);
  }

  public void periodic() {
    io.updateInputs(inputs);

    Logger.processInputs("CoralIntake", inputs);

    Logger.recordOutput("CoralIntake/CoralIntakeMotorConnected", inputs.velocityRadsPerSec != 0);
  }

  @AutoLogOutput(key = "CoralIntake/Close")
  public boolean isVoltageClose(double setVoltage) {
    double voltageDifference = Math.abs(setVoltage - inputs.appliedVoltage);
    return voltageDifference <= CoralIntakeConstants.CORAL_INTAKE_TOLERANCE;
  }

  public void setVoltage(double voltage) {
    io.setVoltage(voltage);
    isVoltageClose(voltage);
  }

  public void setBrake(boolean brake) {
    io.setBrake(brake);
  }

  // Allows manual command of the flywheel for testing
  public Command ManualCommand(DoubleSupplier velocitySupplier) {
    return new FunctionalCommand(
        () -> {},
        () -> setVoltage(velocitySupplier.getAsDouble() * 12),
        (stop) -> setVoltage(0),
        () -> false,
        this);
  }

  public Command ManualCommand(double velocity) {
    return ManualCommand(() -> velocity);
  }
}
