package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import java.util.function.DoubleSupplier;
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

  public static enum State {
    MANUAL,
    PID,
    SYSID
  }

  private State elevatorState = State.MANUAL;
  private double setpoint = 0;
  private double manualSpeed = 0;

  public Elevator(ElevatorIO io) {
    this.io = io;

    SmartDashboard.putData(getName(), this);

    logP = new LoggedNetworkNumber("/SmartDashboard/Elevator/P", io.getP());
    logI = new LoggedNetworkNumber("/SmartDashboard/Elevator/I", io.getI());
    logD = new LoggedNetworkNumber("/SmartDashboard/Elevator/D", io.getD());
    logFF = new LoggedNetworkNumber("/SmartDashboard/Elevator/FF", io.getFF());

    SysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.per(Second).of(ElevatorConstants.RAMP_RATE),
                Volts.of(ElevatorConstants.STEP_VOLTAGE),
                null),
            new SysIdRoutine.Mechanism(
                v -> io.setVoltage(v.in(Volts) / 12.0),
                (sysidLog) -> {
                  sysidLog
                      .motor("Elevator")
                      .voltage(m_appliedVoltage.mut_replace(inputs.appliedVoltage, Volts))
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

    // Move elevator based on state
    switch (elevatorState) {
      case MANUAL:
        move(manualSpeed);
        break;
      case PID:
        runPID();
        break;
      default:
        break;
    }

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
        inputs.positionMeters != ElevatorConstants.ELEVATOR_OFFSET_METERS);
  }

  public void setPID(double setpoint) {
    this.setpoint = setpoint;
    elevatorState = State.PID;
  }

  public void setManual(double speed) {
    manualSpeed = speed;
    if (speed != 0) {
      elevatorState = State.MANUAL;
    }
  }

  public void setMechanism(MechanismLigament2d mechanism) {
    elevatorMechanism = mechanism;
  }

  public MechanismLigament2d append(MechanismLigament2d mechanism) {
    return elevatorMechanism.append(mechanism);
  }

  public MechanismLigament2d getElevatorMechanism() {
    elevatorMechanism = new MechanismLigament2d("Elevator", 0.4, 90, 5, new Color8Bit(Color.kAqua));
    return elevatorMechanism;
  }

  public void setBrake(boolean brake) {
    io.setBrakeMode(brake);
  }

  public boolean atSetpoint() {
    return io.atSetpoint();
  }

  public void move(double speed) {
    if ((io.getPosition() <= ElevatorConstants.ELEVATOR_MIN_HEIGHT && io.getVelocity() < 0)
        || ((io.getPosition() >= ElevatorConstants.ELEVATOR_MAX_HEIGHT || io.isLimitSwitchPressed())
            && io.getVelocity() > 0)) {
      io.setVoltage(0);
    } else {
      io.setVoltage(speed * 12);
    }
  }

  public void runPID() {
    if (setpoint > ElevatorConstants.ELEVATOR_MAX_HEIGHT) {
      setpoint = ElevatorConstants.ELEVATOR_MAX_HEIGHT;
    } else if (setpoint < ElevatorConstants.ELEVATOR_MAX_HEIGHT) {
      setpoint = ElevatorConstants.ELEVATOR_MAX_HEIGHT;
    }
    if ((io.getPosition() <= ElevatorConstants.ELEVATOR_MIN_HEIGHT && io.getVelocity() < 0)
        || ((io.getPosition() >= ElevatorConstants.ELEVATOR_MAX_HEIGHT || io.isLimitSwitchPressed())
            && io.getVelocity() > 0)) {
      io.setVoltage(0);
    } else {
      io.goToSetpoint(setpoint);
    }
  }

  /**
   * Runs PID and stops when at setpoint
   *
   * @param setpoint the setpoint in meters
   */
  public Command PIDCommand(double setpoint) {
    return new RunCommand(() -> setPID(setpoint), this).until(() -> atSetpoint());
  }

  public Command InstantPIDCommand(double setpoint) {
    return new InstantCommand(() -> setPID(setpoint));
  }

  /** Control the elevator by providing a velocity from -1 to 1 */
  public Command ManualCommand(double speed) {
    return new FunctionalCommand(
        () -> move(speed), () -> move(speed), (interrupted) -> move(0), () -> false, this);
  }

  /** Control the elevator by providing a velocity from -1 to 1 */
  public Command ManualCommand(DoubleSupplier speedSupplier) {
    return new RunCommand(() -> setManual(speedSupplier.getAsDouble()), this)
        .andThen(
            () -> {
              manualSpeed = 0;
              move(0);
            });
  }

  public Command quasistaticForward() {
    elevatorState = State.SYSID;
    return SysId.quasistatic(Direction.kForward)
        .until(() -> io.getPosition() > ElevatorConstants.ELEVATOR_MAX_HEIGHT)
        .alongWith(
            new InstantCommand(
                () -> Logger.recordOutput("Elevator/sysid-test-state-", "quasistatic-forward")));
  }

  public Command quasistaticBack() {
    elevatorState = State.SYSID;
    return SysId.quasistatic(Direction.kReverse)
        .until(() -> io.getPosition() > ElevatorConstants.ELEVATOR_MIN_HEIGHT)
        .alongWith(
            new InstantCommand(
                () -> Logger.recordOutput("Elevator/sysid-test-state-", "quasistatic-reverse")));
  }

  public Command dynamicForward() {
    elevatorState = State.SYSID;
    return SysId.dynamic(Direction.kForward)
        .until(() -> io.getPosition() > ElevatorConstants.ELEVATOR_MAX_HEIGHT)
        .alongWith(
            new InstantCommand(
                () -> Logger.recordOutput("Elevator/sysid-test-state-", "dynamic-forward")));
  }

  public Command dynamicBack() {
    elevatorState = State.SYSID;
    return SysId.dynamic(Direction.kReverse)
        .until(() -> io.getPosition() > ElevatorConstants.ELEVATOR_MIN_HEIGHT)
        .alongWith(
            new InstantCommand(
                () -> Logger.recordOutput("Elevator/sysid-test-state-", "dynamic-reverse")));
  }
}
