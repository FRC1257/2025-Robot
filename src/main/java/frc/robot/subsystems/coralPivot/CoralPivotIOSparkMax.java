// Made just to change spark max because commit didn't work on a different laptop and I don't want
// merging issues

package frc.robot.subsystems.coralPivot;

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
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class CoralPivotIOSparkMax implements CoralPivotIO {
  // Motor and Encoders
  private SparkMax pivotMotor;
  private SparkMaxConfig config;

  private final ProfiledPIDController pidController;
  private ArmFeedforward feedforward = new ArmFeedforward(0, 0, 0, 0);

  private SparkAbsoluteEncoder motorEncoder;

  private double setpoint = 0;

  // These variables are used to find the acceleration of the PID setpoint
  // (change in velocity / time = avg acceleration)
  double lastSpeed = 0;
  double lastTime = Timer.getFPGATimestamp();

  public CoralPivotIOSparkMax() {
    pivotMotor = new SparkMax(CoralPivotConstants.CORAL_PIVOT_ID, MotorType.kBrushless);

    config = new SparkMaxConfig();

    config
        .idleMode(IdleMode.kBrake)
        .voltageCompensation(12.0)
        .smartCurrentLimit(Constants.NEO_VORTEX_CURRENT_LIMIT);

    motorEncoder = pivotMotor.getAbsoluteEncoder();

    config
        .absoluteEncoder
        .zeroCentered(true)
        .zeroOffset(CoralPivotConstants.CORAL_PIVOT_OFFSET)
        .positionConversionFactor(2 * Constants.PI)
        .velocityConversionFactor(2 * Constants.PI)
        .inverted(false);
    // .positionConversionFactor(2 * Constants.PI)
    // .velocityConversionFactor(2 * Constants.PI / 60.0)
    // .startPulseUs(1)
    // .endPulseUs(1024);

    pidController =
        new ProfiledPIDController(
            CoralPivotConstants.CORAL_PIVOT_PID_REAL[0],
            CoralPivotConstants.CORAL_PIVOT_PID_REAL[1],
            CoralPivotConstants.CORAL_PIVOT_PID_REAL[2],
            new TrapezoidProfile.Constraints(
                CoralPivotConstants.CORAL_PIVOT_MAX_VELOCITY,
                CoralPivotConstants.CORAL_PIVOT_MAX_ACCELERATION));

    // 0 position for absolute encoder is at 0.2585 rad, so subtract that value from everything

    pivotMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    configureFeedForward();
  }

  private void configureFeedForward() {
    setkS(CoralPivotConstants.CORAL_PIVOT_FEEDFORWARD_REAL[0]);
    setkG(CoralPivotConstants.CORAL_PIVOT_FEEDFORWARD_REAL[1]);
    setkV(CoralPivotConstants.CORAL_PIVOT_FEEDFORWARD_REAL[2]);
    setkA(CoralPivotConstants.CORAL_PIVOT_FEEDFORWARD_REAL[3]);
  }

  /** Updates the set of loggable inputs. */
  @Override
  public void updateInputs(CoralPivotIOInputs inputs) {

    inputs.data =
        new CoralPivotIOData(
            getAngle(),
            getAngVelocity(),
            pivotMotor.getAppliedOutput() * pivotMotor.getBusVoltage(),
            pidController.getSetpoint().position,
            new double[] {pivotMotor.getOutputCurrent()},
            new double[] {pivotMotor.getMotorTemperature()});
  }

  /** Run open loop at the specified voltage. */
  @Override
  public void setVoltage(double motorVolts) {
    Logger.recordOutput("CoralPivot/Desired Voltage", motorVolts);
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
  }

  @Override
  public void goToSetpoint() {
    double pidOutput = pidController.calculate(getAngle());

    // change in velocity / change in time = acceleration
    // Acceleration is used to calculate feedforward
    double acceleration =
        (pidController.getSetpoint().velocity - lastSpeed) / (Timer.getFPGATimestamp() - lastTime);

    double ffOutput =
        feedforward.calculate(
            pidController.getSetpoint().position,
            pidController.getSetpoint().velocity,
            acceleration);

    setVoltage(MathUtil.clamp(pidOutput + ffOutput, -12, 12));

    lastSpeed = pidController.getSetpoint().velocity;
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
    return Math.abs(getAngle() - setpoint) < CoralPivotConstants.CORAL_PIVOT_PID_TOLERANCE;
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
    return pidController.getP();
  }

  @Override
  public double getI() {
    return pidController.getI();
  }

  @Override
  public double getD() {
    return pidController.getD();
  }
}
