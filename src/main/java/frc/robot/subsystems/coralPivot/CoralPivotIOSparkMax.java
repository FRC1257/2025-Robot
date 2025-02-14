// Made just to change spark max because commit didn't work on a different laptop and I don't want
// merging issues

package frc.robot.subsystems.coralPivot;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class CoralPivotIOSparkMax implements CoralPivotIO {
  // Motor and Encoders
  private SparkMax pivotMotor;
  private SparkMaxConfig config;
  private final SparkClosedLoopController pidController;
  private ArmFeedforward feedforward = new ArmFeedforward(0, 0, 0, 0);

  private RelativeEncoder motorEncoder;
  private DutyCycleEncoder absoluteEncoder;

  private double setpoint = 0;

  private double kP = 0, kI = 0, kD = 0;

  public CoralPivotIOSparkMax() {
    pivotMotor = new SparkMax(CoralPivotConstants.CORAL_PIVOT_ID, MotorType.kBrushless);

    config = new SparkMaxConfig();

    setBrake(true);

    config.voltageCompensation(12.0).smartCurrentLimit(Constants.NEO_CURRENT_LIMIT);

    motorEncoder = pivotMotor.getEncoder();

    config
        .encoder
        .positionConversionFactor(2 * Constants.PI * CoralPivotConstants.POSITION_CONVERSION_FACTOR)
        .velocityConversionFactor(
            2 * Constants.PI * CoralPivotConstants.POSITION_CONVERSION_FACTOR / 60.0);

    // absoluteEncoder.reset();
    // make sure the pivot starts at the bottom position every time
    // absoluteEncoder.reset();

    pidController = pivotMotor.getClosedLoopController();

    config.closedLoop.pid(
        CoralPivotConstants.CORAL_PIVOT_PID_REAL[0],
        CoralPivotConstants.CORAL_PIVOT_PID_REAL[1],
        CoralPivotConstants.CORAL_PIVOT_PID_REAL[2]);

    config
        .closedLoop
        .maxMotion
        .maxVelocity(CoralPivotConstants.CORAL_PIVOT_MAX_VELOCITY)
        .maxAcceleration(CoralPivotConstants.CORAL_PIVOT_MAX_ACCELERATION);

    // 0 position for absolute encoder is at 0.2585 rad, so subtract that value from everything

    pivotMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    configureFeedForward();

    absoluteEncoder = new DutyCycleEncoder(CoralPivotConstants.ABSOLUTE_ENCODER_CHANNEL);
    absoluteEncoder.setDutyCycleRange(1.0 / 1025.0, 1024.0 / 1025.0);
    absoluteEncoder.setAssumedFrequency(975.6);
    Logger.recordOutput("Absolute Encoder Starting Position: ", absoluteEncoder.get());

    motorEncoder.setPosition(getAngle());
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
    inputs.angleRads = getAngle();
    Logger.recordOutput("CoralPivot/RelativePosition", motorEncoder.getPosition());
    inputs.angVelocityRadsPerSec = motorEncoder.getVelocity();
    inputs.appliedVolts = pivotMotor.getAppliedOutput() * pivotMotor.getBusVoltage();
    inputs.currentAmps = new double[] {pivotMotor.getOutputCurrent()};
    inputs.tempCelsius = new double[] {pivotMotor.getMotorTemperature()};
    inputs.setpointAngleRads = setpoint;
  }

  /** Run open loop at the specified voltage. */
  @Override
  public void setVoltage(double motorVolts) {
    Logger.recordOutput("CoralPivot/AppliedVolts", motorVolts);
    pivotMotor.setVoltage(motorVolts);
  }

  /** Returns the current distance measurement. */
  @Override
  public double getAngle() {
    return absoluteEncoder.get() * CoralPivotConstants.POSITION_CONVERSION_FACTOR
        + CoralPivotConstants.CORAL_PIVOT_OFFSET;
  }

  @Override
  public double getAngVelocity() {
    return motorEncoder.getVelocity();
  }

  /** Go to Setpoint */
  @Override
  public void goToSetpoint(double setpoint) {
    // With the setpoint value we run PID control like normal
    double feedforwardOutput = feedforward.calculate(getAngle(), 0);

    Logger.recordOutput("CoralPivot/FeedforwardOutput", feedforwardOutput);

    pidController.setReference(
        setpoint, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0, feedforwardOutput);
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
    config.closedLoop.p(p);
    pivotMotor.configure(
        config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    kP = p;
  }

  @Override
  public void setI(double i) {
    config.closedLoop.i(i);
    pivotMotor.configure(
        config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    kI = i;
  }

  @Override
  public void setD(double d) {
    config.closedLoop.d(d);
    pivotMotor.configure(
        config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    kD = d;
  }

  @Override
  public void setFF(double ff) {
    // pidController.setFF(ff);
  }

  @Override
  public void setkS(double kS) {
    feedforward =
        new ArmFeedforward(kS, feedforward.getKg(), feedforward.getKv(), feedforward.getKa());
  }

  @Override
  public void setkG(double kG) {
    feedforward =
        new ArmFeedforward(feedforward.getKs(), kG, feedforward.getKv(), feedforward.getKa());
  }

  @Override
  public void setkV(double kV) {
    feedforward =
        new ArmFeedforward(feedforward.getKs(), feedforward.getKg(), kV, feedforward.getKa());
  }

  @Override
  public void setkA(double kA) {
    feedforward =
        new ArmFeedforward(feedforward.getKs(), feedforward.getKg(), feedforward.getKv(), kA);
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
  public double getFF() {
    // return pidController.getFF();
    return 0;
  }
}
