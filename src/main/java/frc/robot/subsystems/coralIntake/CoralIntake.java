package frc.robot.subsystems.coralIntake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class CoralIntake extends SubsystemBase {
  private final CoralIntakeIO io;
  CoralIntakeIOInputsAutoLogged inputs = new CoralIntakeIOInputsAutoLogged();

  private LoggedNetworkNumber logP;
  private LoggedNetworkNumber logI;
  private LoggedNetworkNumber logD;

  public CoralIntake(CoralIntakeIO io) {
    this.io = io;
    SmartDashboard.putData(getName(), this);

    logP = new LoggedNetworkNumber("/SmartDashboard/CoralIntake/P", io.getP());
    logI = new LoggedNetworkNumber("/SmartDashboard/CoralIntake/I", io.getI());
    logD = new LoggedNetworkNumber("/SmartDashboard/CoralIntake/D", io.getD());
  }

  public void periodic() {
    io.updateInputs(inputs);
    // Update PID constants to ensure they are up to date
    if (logP.get() != io.getP()) {
      io.setP(logP.get());
    }
    if (logI.get() != io.getI()) {
      io.setI(logI.get());
    }
    if (logD.get() != io.getD()) {
      io.setD(logD.get());
    }
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

  /**
   * Uses input from controller to set speed of the flywheel and is used as the default command for
   * the intake
   */
  public Command speedCommand(DoubleSupplier speed) {
    return new FunctionalCommand(
        () -> {}, () -> io.setSpeed(speed.getAsDouble()), (stop) -> io.stop(), () -> false, this);
  }
  // Allows manual command of the flywheel for testing
  public Command manualCommand(DoubleSupplier voltage) {
    return new FunctionalCommand(
        () -> {},
        () -> io.setVoltage(voltage.getAsDouble()),
        (stop) -> io.stop(),
        () -> false,
        this);
  }

  // Allows manual command of the flywheel for testing
  public Command manualCommand(double voltage) {
    return manualCommand(() -> voltage);
  }

  public Command stop() {
    return new FunctionalCommand(
        () -> {}, () -> io.setVoltage(0), (stop) -> io.stop(), () -> false, this);
  }
}
