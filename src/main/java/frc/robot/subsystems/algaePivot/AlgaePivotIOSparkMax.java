// Made just to change spark max because commit didn't work on a different laptop and I don't want
// merging issues

package frc.robot.subsystems.algaePivot;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class AlgaePivotIOSparkMax implements AlgaePivotIO {
  // Motor and Encoders
  private SparkMax pivotMotor;
  private SparkMaxConfig config;
  private final ProfiledPIDController pidController;
  private final ProfiledPIDController pidControllerActive;
  private ArmFeedforward feedforward = new ArmFeedforward(0, 0, 0, 0);
  private ArmFeedforward feedforwardActive = new ArmFeedforward(0, 0, 0, 0);

  private SparkAbsoluteEncoder motorEncoder;

  private DigitalInput breakBeam;

  private double setpoint = 0;

  private double kP = AlgaePivotConstants.ALGAE_PIVOT_PID_REAL[0],
      kI = AlgaePivotConstants.ALGAE_PIVOT_PID_REAL[1],
      kD = AlgaePivotConstants.ALGAE_PIVOT_PID_REAL[2];
  private double kActiveP = AlgaePivotConstants.ALGAE_PIVOT_PID_REAL_ACTIVE[0],
      kActiveI = AlgaePivotConstants.ALGAE_PIVOT_PID_REAL_ACTIVE[1],
      kActiveD = AlgaePivotConstants.ALGAE_PIVOT_PID_REAL_ACTIVE[2];

  // These variables are used to find the acceleration of the PID setpoint
  // (change in velocity / change in time = avg acceleration)
  double lastSpeed = 0;
  double lastTime = Timer.getFPGATimestamp();

  public AlgaePivotIOSparkMax() {
    pivotMotor = new SparkMax(AlgaePivotConstants.ALGAE_PIVOT_ID, MotorType.kBrushless);

    config = new SparkMaxConfig();

    config
        .idleMode(IdleMode.kBrake)
        .voltageCompensation(12.0)
        .smartCurrentLimit(Constants.NEO_CURRENT_LIMIT)
        .inverted(true);

    motorEncoder = pivotMotor.getAbsoluteEncoder();

    config
        .absoluteEncoder
        .zeroCentered(true)
        .zeroOffset(AlgaePivotConstants.ALGAE_PIVOT_OFFSET)
        .positionConversionFactor(2 * Constants.PI)
        .velocityConversionFactor(2 * Constants.PI);

    // absoluteEncoder.reset();
    // make sure the pivot starts at the bottom position every time
    // absoluteEncoder.reset();

    pidController =
        new ProfiledPIDController(
            kP,
            kI,
            kD,
            new TrapezoidProfile.Constraints(
                AlgaePivotConstants.ALGAE_PIVOT_MAX_VELOCITY,
                AlgaePivotConstants.ALGAE_PIVOT_MAX_ACCELERATION));

    pidControllerActive =
        new ProfiledPIDController(
            kActiveP,
            kActiveI,
            kActiveD,
            new TrapezoidProfile.Constraints(
                AlgaePivotConstants.ALGAE_PIVOT_MAX_VELOCITY,
                AlgaePivotConstants.ALGAE_PIVOT_MAX_ACCELERATION));

    pivotMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    configureFeedForward();
    configureFeedForwardActive();

    breakBeam = new DigitalInput(AlgaePivotConstants.BREAK_BEAM_CHANNEL);
  }

  private void configureFeedForward() {
    setkS(AlgaePivotConstants.ALGAE_PIVOT_FEEDFORWARD_REAL[0]);
    setkG(AlgaePivotConstants.ALGAE_PIVOT_FEEDFORWARD_REAL[1]);
    setkV(AlgaePivotConstants.ALGAE_PIVOT_FEEDFORWARD_REAL[2]);
    setkA(AlgaePivotConstants.ALGAE_PIVOT_FEEDFORWARD_REAL[3]);
  }

  private void configureFeedForwardActive() {
    setActivekS(AlgaePivotConstants.ALGAE_PIVOT_FEEDFORWARD_REAL_ACTIVE[0]);
    setActivekG(AlgaePivotConstants.ALGAE_PIVOT_FEEDFORWARD_REAL_ACTIVE[1]);
    setActivekV(AlgaePivotConstants.ALGAE_PIVOT_FEEDFORWARD_REAL_ACTIVE[2]);
    setActivekA(AlgaePivotConstants.ALGAE_PIVOT_FEEDFORWARD_REAL_ACTIVE[3]);
  }

  /** Updates the set of loggable inputs. */
  @Override
  public void updateInputs(AlgaePivotIOInputs inputs) {
    inputs.data =
        new AlgaePivotIOData(
            getAngle(),
            motorEncoder.getVelocity(),
            pivotMotor.getAppliedOutput() * pivotMotor.getBusVoltage(),
            pidController.getSetpoint().position,
            isBreakBeamBroken(),
            new double[] {pivotMotor.getOutputCurrent()},
            new double[] {pivotMotor.getMotorTemperature()});
  }

  /** Run open loop at the specified voltage. */
  @Override
  public void setVoltage(double motorVolts) {
    Logger.recordOutput("AlgaePivot/Desired Voltage", motorVolts);
    pivotMotor.setVoltage(motorVolts);
  }

  /** Returns the current distance measurement. */
  @Override
  public double getAngle() {
    return motorEncoder.getPosition();
  }

  @Override
  public double getAngVelocity() {
    return motorEncoder.getVelocity();
  }

  @Override
  public void setSetpoint(double setpoint) {
    pidController.setGoal(setpoint);
    pidController.reset(getAngle(), getAngVelocity());
    pidControllerActive.setGoal(setpoint);
    pidControllerActive.reset(getAngle(), getAngVelocity());
    Logger.recordOutput("AlgaePivot/Actual Setpoint", pidController.getSetpoint().position);
  }

  @Override
  public void goToSetpoint() {
    double pidOutput = 0, ffOutput = 0;

    if (isBreakBeamBroken()) {
      pidOutput = pidControllerActive.calculate(getAngle());

      // change in velocity / change in time = acceleration
      // Acceleration is used to calculate feedforward
      double acceleration =
          (pidControllerActive.getSetpoint().velocity - lastSpeed)
              / (Timer.getFPGATimestamp() - lastTime);

      Logger.recordOutput("CoralPivot/Acceleration", acceleration);

      ffOutput =
          feedforwardActive.calculate(
              pidControllerActive.getSetpoint().position,
              pidControllerActive.getSetpoint().velocity,
              acceleration);

      lastSpeed = pidControllerActive.getSetpoint().velocity;
    } else {
      pidOutput = pidController.calculate(getAngle());

      // change in velocity / change in time = acceleration
      // Acceleration is used to calculate feedforward
      double acceleration =
          (pidController.getSetpoint().velocity - lastSpeed)
              / (Timer.getFPGATimestamp() - lastTime);

      Logger.recordOutput("CoralPivot/Acceleration", acceleration);

      ffOutput =
          feedforward.calculate(
              pidController.getSetpoint().position,
              pidController.getSetpoint().velocity,
              acceleration);

      lastSpeed = pidController.getSetpoint().velocity;

      Logger.recordOutput("AlgaePivot/PID output", pidOutput);
      Logger.recordOutput("AlgaePivot/FF output", ffOutput);
    }

    setVoltage(MathUtil.clamp(pidOutput + ffOutput, -12, 12));

    lastTime = Timer.getFPGATimestamp();
  }

  @Override
  public void setBrake(boolean brake) {
    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
    pivotMotor.configure(
        config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public boolean atSetpoint() {
    return Math.abs(getAngle() - setpoint) < AlgaePivotConstants.ALGAE_PIVOT_PID_TOLERANCE;
  }

  @Override
  public boolean isBreakBeamBroken() {
    // return breakBeam.get();
    return false;
  }

  @Override
  public void setP(double p) {
    pidController.setP(p);
  }

  @Override
  public void setI(double i) {
    pidController.setI(i);
  }

  @Override
  public void setD(double d) {
    pidController.setD(d);
  }

  @Override
  public void setkS(double kS) {
    feedforward.setKs(kS);
  }

  @Override
  public void setkG(double kG) {
    feedforward.setKg(kG);
  }

  @Override
  public void setkV(double kV) {
    feedforward.setKv(kV);
  }

  @Override
  public void setkA(double kA) {
    feedforward.setKa(kA);
  }

  @Override
  public double getkS() {
    return feedforward.getKs();
  }

  @Override
  public double getkG() {
    return feedforward.getKg();
  }

  @Override
  public double getkV() {
    return feedforward.getKv();
  }

  @Override
  public double getkA() {
    return feedforward.getKa();
  }

  @Override
  public double getP() {
    return kP;
  }

  @Override
  public double getI() {
    return kI;
  }

  @Override
  public double getD() {
    return kD;
  }

  @Override
  public void setActiveP(double p) {
    pidControllerActive.setP(p);
  }

  @Override
  public void setActiveI(double i) {
    pidControllerActive.setI(i);
  }

  @Override
  public void setActiveD(double d) {
    pidControllerActive.setD(d);
  }

  @Override
  public void setActivekS(double kS) {
    feedforwardActive.setKs(kS);
  }

  @Override
  public void setActivekG(double kG) {
    feedforwardActive.setKg(kG);
  }

  @Override
  public void setActivekV(double kV) {
    feedforwardActive.setKv(kV);
  }

  @Override
  public void setActivekA(double kA) {
    feedforwardActive.setKa(kA);
  }

  @Override
  public double getActivekS() {
    return feedforwardActive.getKs();
  }

  @Override
  public double getActivekG() {
    return feedforwardActive.getKg();
  }

  @Override
  public double getActivekV() {
    return feedforwardActive.getKv();
  }

  @Override
  public double getActivekA() {
    return feedforwardActive.getKa();
  }

  @Override
  public double getActiveP() {
    return pidControllerActive.getP();
  }

  @Override
  public double getActiveI() {
    return pidControllerActive.getI();
  }

  @Override
  public double getActiveD() {
    return pidControllerActive.getD();
  }
}
