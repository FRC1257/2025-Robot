package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class Elevator extends SubsystemBase {
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  private LoggedNetworkNumber logP;
  private LoggedNetworkNumber logI;
  private LoggedNetworkNumber logD;

  private LoggedNetworkNumber logkS;
  private LoggedNetworkNumber logkG;
  private LoggedNetworkNumber logkV;
  private LoggedNetworkNumber logkA;

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
  private double manualSpeed = 0;

  public Elevator(ElevatorIO io) {
    this.io = io;

    SmartDashboard.putData(getName(), this);

    logP = new LoggedNetworkNumber("/SmartDashboard/Elevator/P", io.getP());
    logI = new LoggedNetworkNumber("/SmartDashboard/Elevator/I", io.getI());
    logD = new LoggedNetworkNumber("/SmartDashboard/Elevator/D", io.getD());

    logkS = new LoggedNetworkNumber("/SmartDashboard/Elevator/kS", io.getkS());
    logkG = new LoggedNetworkNumber("/SmartDashboard/Elevator/kG", io.getkG());
    logkV = new LoggedNetworkNumber("/SmartDashboard/Elevator/kV", io.getkV());
    logkA = new LoggedNetworkNumber("/SmartDashboard/Elevator/kA", io.getkA());

    SysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.per(Second).of(ElevatorConstants.SYSID_RAMP_RATE),
                Volts.of(ElevatorConstants.SYSID_STEP_VOLTAGE),
                Seconds.of(ElevatorConstants.SYSID_TIME),
                (state) -> Logger.recordOutput("Elevator/SysIdTestState", state.toString())),
            new SysIdRoutine.Mechanism(
                v -> io.setVoltage(v.in(Volts)),
                (sysidLog) -> {
                  sysidLog
                      .motor("Elevator")
                      .voltage(m_appliedVoltage.mut_replace(inputs.data.appliedVoltage(), Volts))
                      .linearPosition(m_position.mut_replace(inputs.data.positionMeters(), Meters))
                      .linearVelocity(
                          m_velocity.mut_replace(
                              inputs.data.velocityMetersPerSec(), MetersPerSecond));
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

    if (logkS.get() != io.getkS()) io.setkS(logkS.get());

    if (logkG.get() != io.getkG()) io.setkG(logkG.get());

    if (logkV.get() != io.getkV()) io.setkV(logkV.get());

    if (logkA.get() != io.getkA()) io.setkA(logkA.get());

    // Log Inputs
    Logger.processInputs("Elevator", inputs);
    Logger.recordOutput("Elevator/Elevator State", elevatorState);
  }

  public void setPID(double setpoint) {
    if (setpoint > ElevatorConstants.ELEVATOR_MAX_HEIGHT)
      setpoint = ElevatorConstants.ELEVATOR_MAX_HEIGHT;
    else if (setpoint < ElevatorConstants.ELEVATOR_MIN_HEIGHT)
      setpoint = ElevatorConstants.ELEVATOR_MIN_HEIGHT;

    io.setSetpoint(setpoint);
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

  @AutoLogOutput(key = "Elevator/Is Voltage Close")
  public boolean isVoltageClose(double setVoltage) {
    double voltageDifference = Math.abs(setVoltage - inputs.data.appliedVoltage());
    return voltageDifference <= ElevatorConstants.ELEVATOR_VOLTAGE_TOLERANCE;
  }

  public void move(double speed) {
    if ((io.getPosition() <= ElevatorConstants.ELEVATOR_MIN_HEIGHT && speed < 0)
        || ((io.getPosition() >= ElevatorConstants.ELEVATOR_MAX_HEIGHT || io.isLimitSwitchPressed())
            && speed > 0)) {
      io.setVoltage(0);
      isVoltageClose(0);
    } else {
      io.setVoltage(speed * 12);
      isVoltageClose(speed * 12);
    }
  }

  public void runPID() {
    if ((io.getPosition() <= ElevatorConstants.ELEVATOR_MIN_HEIGHT && io.getVelocity() < -0.1)
        || ((io.getPosition() >= ElevatorConstants.ELEVATOR_MAX_HEIGHT || io.isLimitSwitchPressed())
            && io.getVelocity() > 0.1)) {
      io.setVoltage(0);
    } else {
      io.goToSetpoint();
    }
  }

  /**
   * Runs PID and stops when at setpoint
   *
   * @param setpoint the setpoint in meters
   */
  public Command PIDCommand(double setpoint) {
    return new InstantCommand(() -> setPID(setpoint), this)
        .andThen(new WaitUntilCommand(() -> atSetpoint()));
  }

  public Command InstantPIDCommand(double setpoint) {
    return new InstantCommand(() -> setPID(setpoint));
  }

  /** Control the elevator by providing a velocity from -1 to 1 */
  public Command ManualCommand(double speed) {
    return new RunCommand(() -> setManual(speed), this)
        .finallyDo(
            () -> {
              manualSpeed = 0;
              move(0);
            });
  }

  /** Control the elevator by providing a velocity from -1 to 1 */
  public Command ManualCommand(DoubleSupplier speedSupplier) {
    return new RunCommand(() -> setManual(speedSupplier.getAsDouble()), this)
        .finallyDo(
            () -> {
              manualSpeed = 0;
              move(0);
            });
  }

  public Command quasistaticForward() {
    elevatorState = State.SYSID;
    return SysId.quasistatic(Direction.kForward)
        .until(
            () ->
                io.getPosition() >= ElevatorConstants.ELEVATOR_MAX_HEIGHT
                    || io.isLimitSwitchPressed());
  }

  public Command quasistaticBack() {
    elevatorState = State.SYSID;
    return SysId.quasistatic(Direction.kReverse)
        .until(() -> io.getPosition() <= ElevatorConstants.ELEVATOR_MIN_HEIGHT);
  }

  public Command dynamicForward() {
    elevatorState = State.SYSID;
    return SysId.dynamic(Direction.kForward)
        .until(
            () ->
                io.getPosition() >= ElevatorConstants.ELEVATOR_MAX_HEIGHT
                    || io.isLimitSwitchPressed());
  }

  public Command dynamicBack() {
    elevatorState = State.SYSID;
    return SysId.dynamic(Direction.kReverse)
        .until(() -> io.getPosition() <= ElevatorConstants.ELEVATOR_MIN_HEIGHT);
  }
}
