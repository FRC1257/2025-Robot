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

  private double setpoint = 0;

  private final ElevatorIO io;

  // Create a Mechanism2d visualization of the arm
  private MechanismLigament2d armMechanism = getArmMechanism();

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
                sysidLog.motor("pivot")
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

    armMechanism.setLength(inputs.positionMeters);

    // Update the PID constants if they have changed
    if (logP.get() != io.getP()) io.setP(logP.get());

    if (logI.get() != io.getI()) io.setI(logI.get());

    if (logD.get() != io.getD()) io.setD(logD.get());

    if (logFF.get() != io.getFF()) io.setFF(logFF.get());

    // Log Inputs
    Logger.processInputs("Elevator", inputs);

    Logger.recordOutput(
        "Elevator/PivotAbsoluteEncoderConnected",
        inputs.positionMeters != ElevatorConstants.ELEVATOR_OFFSET);
  }

  public void setBrake(boolean brake) {
    io.setBrakeMode(brake);
  }

  @AutoLogOutput(key = "Elevator/Close")
  public boolean isVoltageClose(double setVoltage) {
    double voltageDifference = Math.abs(setVoltage - inputs.appliedVoltage);
    return voltageDifference <= ElevatorConstants.ELEVATOR_TOLERANCE;
  }

  public void setVelocity(double velocity) {
    if (io.getPosition() > ElevatorConstants.ELEVATOR_MAX_ANGLE && velocity > 0) {
      velocity = 0;
    } else if (io.getPosition() < ElevatorConstants.ELEVATOR_MIN_ANGLE && velocity < 0) {
      velocity = 0;
    }

    isVoltageClose(velocity);
  }

  public void setPID(double setpoint) {
    this.setpoint = setpoint;
    Logger.recordOutput("Elevator/Setpoint", setpoint);
  }

  public boolean atSetpoint() {
    return Math.abs(io.getPosition() - setpoint) < ElevatorConstants.ELEVATOR_PID_TOLERANCE
        && Math.abs(getVelocity()) < ElevatorConstants.ELEVATOR_PID_VELOCITY_TOLERANCE;
  }

  public void setMechanism(MechanismLigament2d mechanism) {
    armMechanism = mechanism;
  }

  public Rotation2d getAngle() {
    return new Rotation2d(inputs.positionMeters);
  }

  public double getVelocity() {
    return inputs.velocityMetersPerSec;
  }

  public Rotation2d getSetpoint() {
    return new Rotation2d(setpoint);
  }

  public MechanismLigament2d append(MechanismLigament2d mechanism) {
    return armMechanism.append(mechanism);
  }

  public MechanismLigament2d getArmMechanism() {
    return new MechanismLigament2d("Pivot Arm", 0.4, 0, 5, new Color8Bit(Color.kAqua));
  }

  public Command PIDCommandForever(DoubleSupplier setpointSupplier) {
    return new FunctionalCommand(
        () -> setPID(setpointSupplier.getAsDouble()),
        () -> {
          setPID(setpointSupplier.getAsDouble());
          io.goToSetpoint(setpoint);
        },
        (stop) -> {},
        () -> false,
        this);
  }

  public Command PIDCommandForever(double setpoint) {
    return new FunctionalCommand(
        () -> setPID(setpoint),
        () -> io.goToSetpoint(setpoint),
        (stop) -> {},
        () -> false,
        this);
  }

  public Command PIDCommand(DoubleSupplier setpointSupplier) {
    return new FunctionalCommand(
        () -> setPID(setpointSupplier.getAsDouble()),
        () -> {
          Logger.recordOutput("ElevatorSpeakerAngle", setpointSupplier.getAsDouble());
          setPID(setpointSupplier.getAsDouble());
          io.goToSetpoint(setpoint);
        },
        (stop) -> setVelocity(0),
        this::atSetpoint,
        this);
  }

  public Command PIDCommand(double setpoint) {
    return new FunctionalCommand(
        () -> setPID(setpoint), () -> io.goToSetpoint(setpoint), (stop) -> setVelocity(0), io::atSetpoint, this);
  }

  // Allows manual control of the pivot arm for PID tuning
  public Command ManualCommand(DoubleSupplier speedSupplier) {
    return new FunctionalCommand(
        () -> setVelocity(speedSupplier.getAsDouble()),
        () -> setVelocity(speedSupplier.getAsDouble()),
        (stop) -> setVelocity(0),
        () -> false,
        this);
  }

  public Command ManualCommand(double speed) {
    return new FunctionalCommand(
        () -> setVelocity(speed),
        () -> setVelocity(speed),
        (stop) -> setVelocity(0),
        () -> false,
        this);
  }
}
