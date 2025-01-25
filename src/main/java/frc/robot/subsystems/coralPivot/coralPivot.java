package frc.robot.subsystems.coralPivot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class coralPivot extends SubsystemBase {
  private final coralPivotIOInputsAutoLogged inputs = new coralPivotIOInputsAutoLogged();

  private LoggedNetworkNumber logP;
  private LoggedNetworkNumber logI;
  private LoggedNetworkNumber logD;

  private double setpoint = 0;

  private final coralPivotIO io;

  // Create a Mechanism2d visualization of the arm
  private MechanismLigament2d armMechanism = getArmMechanism();

  private SysIdRoutine SysId;

  public coralPivot(coralPivotIO io) {
    this.io = io;

    SmartDashboard.putData(getName(), this);

    logP = new LoggedNetworkNumber("/SmartDashboard/coralPivot/P", io.getP());
    logI = new LoggedNetworkNumber("/SmartDashboard/coralPivot/I", io.getI());
    logD = new LoggedNetworkNumber("/SmartDashboard/coralPivot/D", io.getD());
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("coralPivot", inputs);

    armMechanism.setAngle(Units.radiansToDegrees(inputs.angleRads));

    // Update the PID constants if they have changed
    if (logP.get() != io.getP()) io.setP(logP.get());

    if (logI.get() != io.getI()) io.setI(logI.get());

    if (logD.get() != io.getD()) io.setD(logD.get());

    // Log Inputs
    Logger.processInputs("coralPivot", inputs);

    Logger.recordOutput(
        "coralPivot/PivotAbsoluteEncoderConnected",
        inputs.angleRads != coralPivotConstants.CORAL_PIVOT_OFFSET);
  }

  public void setBrake(boolean brake) {
    io.setBrake(brake);
  }

  @AutoLogOutput(key = "coralPivot/Close")
  public boolean isVoltageClose(double setVoltage) {
    double voltageDifference = Math.abs(setVoltage - inputs.appliedVolts);
    return voltageDifference <= coralPivotConstants.CORAL_PIVOT_TOLERANCE;
  }

  public void setVoltage(double motorVolts) {
    // limit the arm if its past the limit
    if (io.getAngle() > coralPivotConstants.CORAL_PIVOT_MAX_ANGLE && motorVolts > 0) {
      motorVolts = 0;
    } else if (io.getAngle() < coralPivotConstants.CORAL_PIVOT_MIN_ANGLE && motorVolts < 0) {
      motorVolts = 0;
    }

    isVoltageClose(motorVolts);
  }

  public void move(double speed) {
    setVoltage(speed);
  }

  public void runPID() {
    io.goToSetpoint(setpoint);
  }

  public void setPID(double setpoint) {
    this.setpoint = setpoint;
    Logger.recordOutput("coralPivot/Setpoint", setpoint);
  }

  public boolean atSetpoint() {
    return Math.abs(io.getAngle() - setpoint) < coralPivotConstants.CORAL_PIVOT_PID_TOLERANCE
        && Math.abs(inputs.angVelocityRadsPerSec)
            < coralPivotConstants.CORAL_PIVOT_PID_VELOCITY_TOLERANCE;
  }

  public void setMechanism(MechanismLigament2d mechanism) {
    armMechanism = mechanism;
  }

  public MechanismLigament2d append(MechanismLigament2d mechanism) {
    return armMechanism.append(mechanism);
  }

  public MechanismLigament2d getArmMechanism() {
    return new MechanismLigament2d("Coral Pivot", 0.4, 0, 5, new Color8Bit(Color.kAqua));
  }

  public Command PIDCommand(DoubleSupplier setpointSupplier) {
    return new FunctionalCommand(
        () -> setPID(setpointSupplier.getAsDouble()),
        () -> {
          Logger.recordOutput("coralPivotSpeakerAngle", setpointSupplier.getAsDouble());
          setPID(setpointSupplier.getAsDouble());
          runPID();
        },
        (stop) -> move(0),
        this::atSetpoint,
        this);
  }

  public Command PIDCommand(double setpoint) {
    return PIDCommand(() -> setpoint);
  }

  public Command PIDCommandForever(DoubleSupplier setpointSupplier) {
    return new FunctionalCommand(
        () -> setPID(setpointSupplier.getAsDouble()),
        () -> {
          setPID(setpointSupplier.getAsDouble());
          runPID();
        },
        (stop) -> move(0),
        () -> false,
        this);
  }

  public Command PIDCommandForever(double setpoint) {
    return PIDCommandForever(() -> setpoint);
  }

  // Allows manual control of the pivot arm for PID tuning
  public Command ManualCommand(DoubleSupplier speedSupplier) {
    return new FunctionalCommand(
        () -> move(speedSupplier.getAsDouble()),
        () -> move(speedSupplier.getAsDouble()),
        (stop) -> move(0),
        () -> false,
        this);
  }
}
