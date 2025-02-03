package frc.robot.subsystems.coralPivot;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

public class CoralPivot extends SubsystemBase {
  private final CoralPivotIOInputsAutoLogged inputs = new CoralPivotIOInputsAutoLogged();

  private LoggedDashboardNumber logP;
  private LoggedDashboardNumber logI;
  private LoggedDashboardNumber logD;
  private LoggedDashboardNumber logFF;

  private LoggedDashboardNumber logkS;
  private LoggedDashboardNumber logkG;
  private LoggedDashboardNumber logkV;
  private LoggedDashboardNumber logkA;

  // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
  private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
  // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
  private final MutableMeasure<Angle> m_angle = mutable(Rotations.of(0));
  // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
  private final MutableMeasure<Velocity<Angle>> m_velocity = mutable(RotationsPerSecond.of(0));

  private double setpoint = 0;

  private final CoralPivotIO io;

  // Create a Mechanism2d visualization of the arm
  private MechanismLigament2d armMechanism = getArmMechanism();

  private SysIdRoutine SysId;

  public CoralPivot(CoralPivotIO io) {
    this.io = io;

    SmartDashboard.putData(getName(), this);

    logP = new LoggedDashboardNumber("CoralPivot/P", io.getP());
    logI = new LoggedDashboardNumber("CoralPivot/I", io.getI());
    logD = new LoggedDashboardNumber("CoralPivot/D", io.getD());
    logFF = new LoggedDashboardNumber("CoralPivot/FF", io.getFF());

    logkS = new LoggedDashboardNumber("CoralPivot/kS", io.getkS());
    logkG = new LoggedDashboardNumber("CoralPivot/kG", io.getkG());
    logkV = new LoggedDashboardNumber("CoralPivot/kV", io.getkV());
    logkA = new LoggedDashboardNumber("CoralPivot/kG", io.getkA());

    SysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.per(Second).of(CoralPivotConstants.RAMP_RATE),
                Volts.of(CoralPivotConstants.STEP_VOLTAGE),
                null),
            new SysIdRoutine.Mechanism(
                v -> io.setVoltage(v.in(Volts)),
                (sysidLog) -> {
                  sysidLog
                      .motor("pivot")
                      .voltage(m_appliedVoltage.mut_replace(inputs.appliedVolts, Volts))
                      .angularPosition(m_angle.mut_replace(inputs.angleRads, Rotations))
                      .angularVelocity(
                          m_velocity.mut_replace(inputs.angVelocityRadsPerSec, RotationsPerSecond));
                },
                this));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("CoralPivot", inputs);

    armMechanism.setAngle(Units.radiansToDegrees(inputs.angleRads));

    // Update the PID constants if they have changed
    if (logP.get() != io.getP()) io.setP(logP.get());

    if (logI.get() != io.getI()) io.setI(logI.get());

    if (logD.get() != io.getD()) io.setD(logD.get());

    if (logFF.get() != io.getFF()) io.setFF(logFF.get());

    if (logkS.get() != io.getkS()) io.setkS(logkS.get());

    if (logkG.get() != io.getkG()) io.setkG(logkG.get());

    if (logkV.get() != io.getkV()) io.setkV(logkV.get());

    if (logkA.get() != io.getkA()) io.setkG(logkA.get());

    // Log Inputs
    Logger.processInputs("CoralPivot", inputs);

    Logger.recordOutput(
        "CoralPivot/PivotAbsoluteEncoderConnected",
        inputs.angleRads != CoralPivotConstants.CORAL_PIVOT_OFFSET);
  }

  public void setBrake(boolean brake) {
    io.setBrake(brake);
  }

  @AutoLogOutput(key = "CoralPivot/Close")
  public boolean isVoltageClose(double setVoltage) {
    double voltageDifference = Math.abs(setVoltage - inputs.appliedVolts);
    return voltageDifference <= CoralPivotConstants.CORAL_PIVOT_TOLERANCE;
  }

  public void setVoltage(double motorVolts) {
    // limit the arm if its past the limit
    if (io.getAngle() > CoralPivotConstants.CORAL_PIVOT_MAX_ANGLE && motorVolts > 0) {
      motorVolts = 0;
    } else if (io.getAngle() < CoralPivotConstants.CORAL_PIVOT_MIN_ANGLE && motorVolts < 0) {
      motorVolts = 0;
    }

    if (DashboardValues.turboMode.get()) {
      io.setVoltage(0);
    } else {
      io.setVoltage(motorVolts);
    }

    isVoltageClose(motorVolts);
  }

  public void runPID() {
    io.goToSetpoint(setpoint);
  }

  public void setPID(double setpoint) {
    this.setpoint = setpoint;
    Logger.recordOutput("CoralPivot/Setpoint", setpoint);
  }

  public void addPID(double setpointAdd) {
    this.setpoint += setpointAdd;
    this.setpoint =
        MathUtil.clamp(
            this.setpoint,
            CoralPivotConstants.CORAL_PIVOT_MIN_ANGLE,
            CoralPivotConstants.CORAL_PIVOT_MAX_ANGLE);

    Logger.recordOutput("CoralPivot/Setpoint", setpoint);
  }

  public boolean atSetpoint() {
    return Math.abs(io.getAngle() - setpoint) < CoralPivotConstants.CORAL_PIVOT_PID_TOLERANCE
        && Math.abs(getVelocity()) < CoralPivotConstants.CORAL_PIVOT_PID_VELOCITY_TOLERANCE;
  }

  public void setMechanism(MechanismLigament2d mechanism) {
    armMechanism = mechanism;
  }

  public Rotation2d getAngle() {
    return new Rotation2d(inputs.angleRads);
  }

  public double getVelocity() {
    return inputs.angVelocityRadsPerSec;
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

  public Command PIDCommand(double setpoint) {
    return new FunctionalCommand(
        () -> setPID(setpoint), () -> runPID(), (stop) -> setVoltage(0), this::atSetpoint, this);
  }

  public Command PIDCommandForever(DoubleSupplier setpointSupplier) {
    return new FunctionalCommand(
        () -> setPID(setpointSupplier.getAsDouble()),
        () -> {
          setPID(setpointSupplier.getAsDouble());
          runPID();
        },
        (stop) -> setVoltage(0),
        () -> false,
        this);
  }

  public Command PIDCommandForever(double setpoint) {
    return new FunctionalCommand(
        () -> setPID(setpoint), () -> runPID(), (stop) -> setVoltage(0), () -> false, this);
  }

  public Command PIDHoldCommand() {
    return new FunctionalCommand(
        () -> setPID(getAngle().getRadians()),
        () -> holdPID(),
        (stop) -> setVoltage(0),
        () -> false,
        this);
  }

  public Command PIDCommand(DoubleSupplier setpointSupplier) {
    return new FunctionalCommand(
        () -> setPID(setpointSupplier.getAsDouble()),
        () -> {
          Logger.recordOutput("CoralPivotSpeakerAngle", setpointSupplier.getAsDouble());
          setPID(setpointSupplier.getAsDouble());
          runPID();
        },
        (stop) -> setVoltage(0),
        this::atSetpoint,
        this);
  }

  // Allows manual control of the pivot arm for PID tuning
  public Command ManualCommand(DoubleSupplier speedSupplier) {
    return new FunctionalCommand(
        () -> setVoltage(speedSupplier.getAsDouble()),
        () -> setVoltage(speedSupplier.getAsDouble()),
        (stop) -> setVoltage(0),
        () -> false,
        this);
  }

  public Command stop() {
    return new FunctionalCommand(
        () -> {}, () -> io.setVoltage(0), (stop) -> io.stop(), () -> false, this);
  }
  // no commmand yalee

  public Command bringDownCommand() {
    return new FunctionalCommand(
        () -> {},
        () -> {
          setVoltage(-1);
          setpoint = 0;
        },
        (interrupted) -> {
          setVoltage(0);
        },
        () -> {
          return io.getAngle() < 0.1;
        },
        this);
  }

  public Command quasistaticForward() {
    return SysId.quasistatic(Direction.kForward)
        .until(() -> getAngle().getRadians() > CoralPivotConstants.CORAL_PIVOT_MAX_ANGLE)
        .alongWith(
            new InstantCommand(
                () -> Logger.recordOutput("CoralPivot/sysid-test-state-", "quasistatic-forward")));
  }

  public Command quasistaticBack() {
    return SysId.quasistatic(Direction.kReverse)
        .until(() -> getAngle().getRadians() < CoralPivotConstants.CORAL_PIVOT_MIN_ANGLE)
        .alongWith(
            new InstantCommand(
                () -> Logger.recordOutput("CoralPivot/sysid-test-state-", "quasistatic-reverse")));
  }

  public Command dynamicForward() {
    return SysId.dynamic(Direction.kForward)
        .until(() -> getAngle().getRadians() > CoralPivotConstants.CORAL_PIVOT_MAX_ANGLE)
        .alongWith(
            new InstantCommand(
                () -> Logger.recordOutput("CoralPivot/sysid-test-state-", "dynamic-forward")));
  }

  public Command dynamicBack() {
    return SysId.dynamic(Direction.kReverse)
        .until(() -> getAngle().getRadians() < CoralPivotConstants.CORAL_PIVOT_MIN_ANGLE)
        .alongWith(
            new InstantCommand(
                () -> Logger.recordOutput("CoralPivot/sysid-test-state-", "dynamic-reverse")));
  }
}
