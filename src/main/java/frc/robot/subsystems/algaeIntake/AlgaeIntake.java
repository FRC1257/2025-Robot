package frc.robot.subsystems.algaeIntake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class AlgaeIntake extends SubsystemBase {
  private final AlgaeIntakeIO io;
  AlgaeIntakeIOInputsAutoLogged inputs = new AlgaeIntakeIOInputsAutoLogged();

  public AlgaeIntake(AlgaeIntakeIO io) {
    this.io = io;
    SmartDashboard.putData(getName(), this);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("AlgaeIntake", inputs);

    Logger.recordOutput("AlgaeIntake/AIntakeMotorConnected", inputs.velocityRadsPerSec != 0);
  }

  @AutoLogOutput(key = "AlgaeIntake/Close")
  public boolean isVoltageClose(double setVoltage) {
    double voltageDifference = Math.abs(setVoltage - inputs.appliedVoltage);
    return voltageDifference <= AlgaeIntakeConstants.ALGAE_INTAKE_TOLERANCE;
  }

  public void setVoltage(double voltage) {
    io.setVoltage(voltage);
    isVoltageClose(voltage);
  }

  public void setBrake(boolean brake) {
    io.setBrake(brake);
  }

  // Allows manual command of the flywheel for testing
  public Command manualCommand(DoubleSupplier velocitySupplier) {
    return new FunctionalCommand(
        () -> {},
        () -> setVoltage(velocitySupplier.getAsDouble() * 12),
        (stop) -> setVoltage(0),
        () -> false,
        this);
  }

  // Allows manual command of the flywheel for testing
  public Command manualCommand(double velocity) {
    return manualCommand(() -> velocity);
  }
}
