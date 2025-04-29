package frc.robot.subsystems.algaePivot;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
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

public class AlgaePivot extends SubsystemBase {
  private final AlgaePivotIOInputsAutoLogged inputs = new AlgaePivotIOInputsAutoLogged();

  private LoggedNetworkNumber logP;
  private LoggedNetworkNumber logI;
  private LoggedNetworkNumber logD;

  private LoggedNetworkNumber logkS;
  private LoggedNetworkNumber logkG;
  private LoggedNetworkNumber logkV;
  private LoggedNetworkNumber logkA;

  private LoggedNetworkNumber logActiveP;
  private LoggedNetworkNumber logActiveI;
  private LoggedNetworkNumber logActiveD;

  private LoggedNetworkNumber logActivekS;
  private LoggedNetworkNumber logActivekG;
  private LoggedNetworkNumber logActivekV;
  private LoggedNetworkNumber logActivekA;

  // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
  private final MutVoltage m_appliedVoltage = Volts.mutable(0);
  // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
  private final MutAngle m_angle = Radians.mutable(0);
  // Mutable holder for unit-safe linear velocity values, persisted to avoid reall?ocation.
  private final MutAngularVelocity m_velocity = RotationsPerSecond.mutable(0);

  private double setpoint = 0;

  private final AlgaePivotIO io;

  // Create a Mechanism2d visualization of the arm
  private MechanismLigament2d armMechanism = getArmMechanism();

  private SysIdRoutine SysId;

  public static enum State {
    MANUAL,
    PID,
    SYSID
  }

  private State armState = State.MANUAL;
  private double manualSpeed = 0;

  public AlgaePivot(AlgaePivotIO io) {
    this.io = io;

    SmartDashboard.putData(getName(), this);

    logP = new LoggedNetworkNumber("/SmartDashboard/AlgaePivot/P", io.getP());
    logI = new LoggedNetworkNumber("/SmartDashboard/AlgaePivot/I", io.getI());
    logD = new LoggedNetworkNumber("/SmartDashboard/AlgaePivot/D", io.getD());

    logkS = new LoggedNetworkNumber("/SmartDashboard/AlgaePivot/kS", io.getkS());
    logkG = new LoggedNetworkNumber("/SmartDashboard/AlgaePivot/kG", io.getkG());
    logkV = new LoggedNetworkNumber("/SmartDashboard/AlgaePivot/kV", io.getkV());
    logkA = new LoggedNetworkNumber("/SmartDashboard/AlgaePivot/kA", io.getkA());

    logActiveP = new LoggedNetworkNumber("/SmartDashboard/AlgaePivot/Active P", io.getP());
    logActiveI = new LoggedNetworkNumber("/SmartDashboard/AlgaePivot/Active I", io.getI());
    logActiveD = new LoggedNetworkNumber("/SmartDashboard/AlgaePivot/Active D", io.getD());

    logActivekS = new LoggedNetworkNumber("/SmartDashboard/AlgaePivot/Active kS", io.getkS());
    logActivekG = new LoggedNetworkNumber("/SmartDashboard/AlgaePivot/Active kG", io.getkG());
    logActivekV = new LoggedNetworkNumber("/SmartDashboard/AlgaePivot/Active kV", io.getkV());
    logActivekA = new LoggedNetworkNumber("/SmartDashboard/AlgaePivot/Active kA", io.getkA());

    SysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.per(Second).of(AlgaePivotConstants.SYSID_RAMP_RATE),
                Volts.of(AlgaePivotConstants.SYSID_STEP_VOLTAGE),
                Seconds.of(AlgaePivotConstants.SYSID_TIME),
                (state) -> Logger.recordOutput("/AlgaePivot/SysIdTestState", state.toString())),
            new SysIdRoutine.Mechanism(
                v -> io.setVoltage(v.in(Volts)),
                (sysidLog) -> {
                  sysidLog
                      .motor("pivot")
                      .voltage(m_appliedVoltage.mut_replace(inputs.data.appliedVolts(), Volts))
                      .angularPosition(m_angle.mut_replace(inputs.data.angleRads(), Rotations))
                      .angularVelocity(
                          m_velocity.mut_replace(
                              inputs.data.angVelocityRadsPerSec(), RotationsPerSecond));
                },
                this));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(getName(), inputs);
    ;

    armMechanism.setAngle(Units.radiansToDegrees(inputs.data.angleRads()));

    // Move arm based on state
    switch (armState) {
      case MANUAL:
        move(manualSpeed);
        break;
      case PID:
        runPID();
        break;
      default:
        break;
    }

    // Update the PID constants if they have changed
    if (logP.get() != io.getP()) io.setP(logP.get());

    if (logI.get() != io.getI()) io.setI(logI.get());

    if (logD.get() != io.getD()) io.setD(logD.get());

    if (logkS.get() != io.getkS()) io.setkS(logkS.get());

    if (logkG.get() != io.getkG()) io.setkG(logkG.get());

    if (logkV.get() != io.getkV()) io.setkV(logkV.get());

    if (logkA.get() != io.getkA()) io.setkA(logkA.get());

    if (logActiveP.get() != io.getActiveP()) io.setActiveP(logActiveP.get());

    if (logActiveI.get() != io.getActiveI()) io.setActiveI(logActiveI.get());

    if (logActiveD.get() != io.getActiveD()) io.setActiveD(logActiveD.get());

    if (logActivekS.get() != io.getActivekS()) io.setActivekS(logActivekS.get());

    if (logActivekG.get() != io.getActivekG()) io.setActivekG(logActivekG.get());

    if (logActivekV.get() != io.getActivekV()) io.setActivekV(logActivekV.get());

    if (logActivekA.get() != io.getActivekA()) io.setActivekA(logActivekA.get());

    // Log Inputs
    Logger.processInputs("AlgaePivot", inputs);

    Logger.recordOutput(
        "AlgaePivot/PivotAbsoluteEncoderConnected",
        inputs.data.angleRads() != AlgaePivotConstants.ALGAE_PIVOT_OFFSET);
  }

  public void setBrake(boolean brake) {
    io.setBrake(brake);
  }

  @AutoLogOutput(key = "AlgaePivot/Is Voltage Close")
  public boolean isVoltageClose(double setVoltage) {
    double voltageDifference = Math.abs(setVoltage - inputs.data.appliedVolts());
    return voltageDifference <= AlgaePivotConstants.ALGAE_PIVOT_TOLERANCE;
  }

  public void move(double speed) {
    // limit the arm if its past the limit
    if (io.getAngle() > AlgaePivotConstants.ALGAE_PIVOT_MAX_ANGLE && speed > 0) {
      speed = 0;
    } else if (io.getAngle() < AlgaePivotConstants.ALGAE_PIVOT_MIN_ANGLE && speed < 0) {
      speed = 0;
    }

    io.setVoltage(speed * 12);

    isVoltageClose(speed * 12);
  }

  public void runPID() {
    if (setpoint > AlgaePivotConstants.ALGAE_PIVOT_MAX_ANGLE) {
      setpoint = AlgaePivotConstants.ALGAE_PIVOT_MAX_ANGLE;
    } else if (setpoint < AlgaePivotConstants.ALGAE_PIVOT_MIN_ANGLE) {
      setpoint = AlgaePivotConstants.ALGAE_PIVOT_MIN_ANGLE;
    }
    if ((io.getAngle() <= AlgaePivotConstants.ALGAE_PIVOT_MIN_ANGLE && io.getAngVelocity() < 0)
        || (io.getAngle() >= AlgaePivotConstants.ALGAE_PIVOT_MAX_ANGLE
            && io.getAngVelocity() > 0)) {
      io.setVoltage(0);
    } else {
      io.goToSetpoint();
    }
  }

  public void setPID(double setpoint) {
    this.setpoint = setpoint;
    armState = State.PID;
    io.setSetpoint(setpoint);
    Logger.recordOutput("AlgaePivot/Setpoint", setpoint);
  }

  public void setManual(double speed) {
    manualSpeed = speed;
    if (speed != 0) {
      armState = State.MANUAL;
    }
  }

  public boolean atSetpoint() {
    return Math.abs(io.getAngle() - setpoint) < AlgaePivotConstants.ALGAE_PIVOT_PID_TOLERANCE
        && Math.abs(io.getAngVelocity()) < AlgaePivotConstants.ALGAE_PIVOT_PID_VELOCITY_TOLERANCE;
  }

  public void setMechanism(MechanismLigament2d mechanism) {
    armMechanism = mechanism;
  }

  public MechanismLigament2d append(MechanismLigament2d mechanism) {
    return armMechanism.append(mechanism);
  }

  public MechanismLigament2d getArmMechanism() {
    armMechanism = new MechanismLigament2d("Algae Pivot", 0.4, 0, 5, new Color8Bit(Color.kAqua));
    return armMechanism;
  }

  public Command PIDCommand(double setpoint) {
    return new InstantCommand(() -> setPID(setpoint), this)
        .andThen(new WaitUntilCommand(() -> atSetpoint()));
  }

  public Command InstantPIDCommand(double setpoint) {
    return new InstantCommand(() -> setPID(setpoint));
  }

  // Allows manual control of the pivot arm for PID tuning
  public Command ManualCommand(DoubleSupplier speedSupplier) {
    return new RunCommand(() -> setManual(speedSupplier.getAsDouble()), this)
        .finallyDo(
            () -> {
              manualSpeed = 0;
              move(0);
            });
  }

  public Command ManualCommand(double speed) {
    return ManualCommand(() -> speed);
  }

  public boolean isBreakBeamBroken() {
    return io.isBreakBeamBroken();
  }

  public Command quasistaticForward() {
    armState = State.SYSID;
    return SysId.quasistatic(Direction.kForward)
        .until(() -> io.getAngle() >= AlgaePivotConstants.ALGAE_PIVOT_MAX_ANGLE);
  }

  public Command quasistaticBack() {
    armState = State.SYSID;
    return SysId.quasistatic(Direction.kReverse)
        .until(() -> io.getAngle() <= AlgaePivotConstants.ALGAE_PIVOT_MIN_ANGLE);
  }

  public Command dynamicForward() {
    armState = State.SYSID;
    return SysId.dynamic(Direction.kForward)
        .until(() -> io.getAngle() >= AlgaePivotConstants.ALGAE_PIVOT_MAX_ANGLE);
  }

  public Command dynamicBack() {
    armState = State.SYSID;
    return SysId.dynamic(Direction.kReverse)
        .until(() -> io.getAngle() <= AlgaePivotConstants.ALGAE_PIVOT_MIN_ANGLE);
  }
}
