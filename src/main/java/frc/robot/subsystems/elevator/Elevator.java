package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.elevator.ElevatorIO.ElevatorIOInputs;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class Elevator extends SubsystemBase {
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  private LoggedNetworkNumber logP;
  private LoggedNetworkNumber logI;
  private LoggedNetworkNumber logD;
  private LoggedNetworkNumber logFF;

  // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
  private final MutVoltage m_appliedVoltage = Volts.mutable(0);
  // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
  private final MutDistance m_position = Meters.mutable(0);
  // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
  private final MutLinearVelocity m_velocity = MetersPerSecond.mutable(0);

  private final ElevatorIO io;

  // Create a Mechanism2d visualization of the elevator
  private MechanismLigament2d elevatorMechanism = getElevatorMechanism();

  private SysIdRoutine SysId;

  public Elevator(ElevatorIO io) {
    this.io = io;

    SmartDashboard.putData(getName(), this);

    logP = new LoggedNetworkNumber("/SmartDashboard/Elevator/P", io.getP());
    logI = new LoggedNetworkNumber("/SmartDashboard/Elevator/I", io.getI());
    logD = new LoggedNetworkNumber("/SmartDashboard/Elevator/D", io.getD());
    logFF = new LoggedNetworkNumber("/SmartDashboard/Elevator/FF", io.getFF());

    SysId = new SysIdRoutine(
        new SysIdRoutine.Config(Volts.per(Second).of(ElevatorConstants.RAMP_RATE), Volts.of(ElevatorConstants.STEP_VOLTAGE), null),
        new SysIdRoutine.Mechanism(v -> io.setVelocity(v.in(Volts) / 12.0), 
            (sysidLog) -> {
                sysidLog.motor("Elevator")
                .voltage(
                    m_appliedVoltage.mut_replace(inputs.appliedVoltage, Volts))
                .linearPosition(m_position.mut_replace(inputs.positionMeters, Meters))
                .linearVelocity(
                    m_velocity.mut_replace(inputs.velocityMetersPerSec, MetersPerSecond));
                    
            }, 
            this));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);

    elevatorMechanism.setLength(io.getPosition());

    // Update the PID constants if they have changed
    if (logP.get() != io.getP()) io.setP(logP.get());

    if (logI.get() != io.getI()) io.setI(logI.get());

    if (logD.get() != io.getD()) io.setD(logD.get());

    if (logFF.get() != io.getFF()) io.setFF(logFF.get());

    // Log Inputs
    Logger.processInputs("Elevator", inputs);

    Logger.recordOutput(
        "Elevator/ElevatorAbsoluteEncoderConnected",
        inputs.positionMeters != ElevatorConstants.ELEVATOR_OFFSET);
  }

  public void setMechanism(MechanismLigament2d mechanism) {
    elevatorMechanism = mechanism;
  }

  public MechanismLigament2d append(MechanismLigament2d mechanism) {
    return elevatorMechanism.append(mechanism);
  }

  public MechanismLigament2d getElevatorMechanism() {
    return new MechanismLigament2d("Elevator", 0.4, 0, 5, new Color8Bit(Color.kAqua));
  }

  public void setBrake(boolean brake) {
    io.setBrakeMode(brake);
  }

  public void setSetpoint(double setpoint) {
    io.goToSetpoint()
  }

  public boolean atSetpoint() {
    io.atSetpoint();
  }

  public void setVelocity(double velocity) {
    if io.getPosition() < ElevatorConstants.ELEVATOR_MIN_HEIGHT || io.getPosition() > ElevatorConstants.ELEVATOR_MAX_HEIGHT {
      io.setVelocity(0);
    } else {
      io.setVelocity(velocity);
    }
  }

  /** Runs PID Command and keeps running it after it reaches setpoint */
  public Command PIDCommandForever(double setpoint) {
    return new FunctionalCommand(
      () -> setSetpoint(setpoint),
      () -> setSetpoint(setpoint),
      (interrupted) -> setVelocity(0),
      () -> false,
      this);
  }
  /** Runs PID Command and keeps running it after it reaches setpoint */
  public Command PIDCommandForever(DoubleSupplier setpointSupplier) {
    return PIDCommandForever(setpointSupplier.getAsDouble());
  }

  /** Runs PID and stops when at setpoint */
  public Command PIDCommand(double setpoint) {
    return new FunctionalCommand(
      () -> setSetpoint(setpoint),
      () -> setSetpoint(setpoint),
      (interrupted) -> setVelocity(0),
      () -> atSetpoint(),
      this);
  }
  /** Runs PID and stops when at setpoint */
  public Command PIDCommand(DoubleSupplier setpointSupplier) {
    return PIDCommand(setpointSupplier.getAsDouble());
  }

  /** Control the elevator by providing a velocity */
  public Command ManualCommand(double speed) {
    return new FunctionalCommand(
      () -> setVelocity(speed),
      () -> setVelocity(speed),
      (interrupted) -> setVelocity(0),
      () -> false,
      this);
  }
  /** Control the elevator by providing a velocity */
  public Command ManualCommand(DoubleSupplier speedSupplier) {
    ManualCommand(speedSupplier.getAsDouble());
  }
}


//add quasistatic and dynamic for sysid later !!!!! 