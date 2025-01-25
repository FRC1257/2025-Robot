package frc.robot.subsystems.coralPivot;

import static frc.robot.Constants.ElectricalLayout.CORAL_PIVOT_ID;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class coralPivotIOSparkMax implements coralPivotIO {
  // Motor and Encoders
  private SparkMax pivotMotor;
  private final SparkClosedLoopController pidController;
  private AbsoluteEncoder encoder;

  // Config
  private SparkMaxConfig config;

  private double setpoint = 0;

  private double kP = coralPivotConstants.CORAL_PIVOT_PID_REAL[0];
  private double kI = coralPivotConstants.CORAL_PIVOT_PID_REAL[1];
  private double kD = coralPivotConstants.CORAL_PIVOT_PID_REAL[2];

  public coralPivotIOSparkMax() {
    pivotMotor = new SparkMax(CORAL_PIVOT_ID, MotorType.kBrushless);

    setBrake(true);

    config.voltageCompensation(12.0).smartCurrentLimit(Constants.NEO_CURRENT_LIMIT);

    config.absoluteEncoder.positionConversionFactor(
        2 * Constants.PI * coralPivotConstants.POSITION_CONVERSION_FACTOR);

    pivotMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // make sure the pivot starts at the bottom position every time
    // absoluteEncoder.reset();

    encoder = pivotMotor.getAbsoluteEncoder();
    pidController = pivotMotor.getClosedLoopController();

    // 0 position for absolute encoder is at 0.2585 rad, so subtract that value from everything

    configurePID();

    // absoluteEncoder.reset();
    Logger.recordOutput("Absolute Encoder Starting Position: ", encoder.getPosition());
  }

  private void configurePID() {
    // pidController.setOutputRange(PivotArmConstants.PIVOT_ARM_MIN_ANGLE,
    // PivotArmConstants.PIVOT_ARM_MAX_ANGLE);
    config.closedLoop.p(coralPivotConstants.CORAL_PIVOT_PID_REAL[0]);
    config.closedLoop.i(coralPivotConstants.CORAL_PIVOT_PID_REAL[1]);
    config.closedLoop.d(coralPivotConstants.CORAL_PIVOT_PID_REAL[2]);
    pivotMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  /** Updates the set of loggable inputs. */
  @Override
  public void updateInputs(coralPivotIOInputs inputs) {
    inputs.angleRads = getAngle();
    inputs.angVelocityRadsPerSec = encoder.getVelocity();
    inputs.appliedVolts = pivotMotor.getAppliedOutput() * pivotMotor.getBusVoltage();
    inputs.currentAmps = new double[] {pivotMotor.getOutputCurrent()};
    inputs.tempCelsius = new double[] {pivotMotor.getMotorTemperature()};
    inputs.setpointAngleRads = setpoint;
  }

  /** Run open loop at the specified voltage. */
  @Override
  public void setVoltage(double motorVolts) {
    Logger.recordOutput("coralPivot/AppliedVolts", motorVolts);
    pivotMotor.setVoltage(motorVolts);
  }

  /** Returns the current distance measurement. */
  @Override
  public double getAngle() {
    return encoder.getPosition() + coralPivotConstants.CORAL_PIVOT_OFFSET;
  }

  /** Go to Setpoint */
  @Override
  public void goToSetpoint(double setpoint) {
    pidController.setReference(setpoint, ControlType.kPosition);
  }

  @Override
  public void setBrake(boolean brake) {
    config.idleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
    pivotMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public boolean atSetpoint() {
    return Math.abs(getAngle() - setpoint) < coralPivotConstants.CORAL_PIVOT_PID_TOLERANCE;
  }

  @Override
  public void setP(double p) {
    config.closedLoop.p(p);
    kP = p;
    pivotMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void setI(double i) {
    config.closedLoop.i(i);
    kI = i;
    pivotMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void setD(double d) {
    config.closedLoop.d(d);
    kD = d;
    pivotMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
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
}
