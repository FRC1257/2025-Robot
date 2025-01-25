package frc.robot.subsystems.coralIntake;

import static frc.robot.Constants.NEO_CURRENT_LIMIT;
import static frc.robot.subsystems.coralIntake.CoralIntakeConstants.CoralIntakePhysicalConstants;
import static frc.robot.subsystems.coralIntake.CoralIntakeConstants.CoralIntakePhysicalConstants.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants.ElectricalLayout;

/** Need to import Constants files/classes */
//

public class CoralIntakeIOSparkMax implements CoralIntakeIO {

  private SparkMax motor;
  private RelativeEncoder encoder;
  private SparkClosedLoopController velocityPID;

  private SparkMaxConfig config;

  private double desiredSpeed;

  private double kP, kI, kD;

  public CoralIntakeIOSparkMax() {
    /** ID needs to be assigned from constants */
    // setPIDConstants(kGroundIntakeP, kGroundIntakeI, kGroundIntakeD);
    motor = new SparkMax(ElectricalLayout.CORAL_INTAKE_MOTOR, MotorType.kBrushless);

    config
        .idleMode(com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kCoast)
        .smartCurrentLimit(NEO_CURRENT_LIMIT)
        .inverted(true);

    encoder = motor.getEncoder();

    velocityPID = motor.getClosedLoopController();

    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    kP = CoralIntakePhysicalConstants.kCoralIntakeP;
    kI = CoralIntakePhysicalConstants.kCoralIntakeI;
    kD = CoralIntakePhysicalConstants.kCoralIntakeD;
  }

  /** updates inputs from robot */
  @Override
  public void updateInputs(CoralIntakeIOInputs inputs) {
    inputs.appliedVoltage = motor.getAppliedOutput() * motor.getBusVoltage();
    inputs.currentAmps = new double[] {motor.getOutputCurrent()};
    inputs.tempCelcius = new double[] {motor.getMotorTemperature()};
    inputs.velocityRadsPerSec = encoder.getVelocity();
    inputs.speedSetpoint = desiredSpeed;
  }

  /** sets voltage to run motor if necessary */
  @Override
  public void setVoltage(double voltage) {
    motor.setVoltage(voltage);
  }

  /** sets brake mode to stop */
  @Override
  public void setBrake(boolean brake) {
    config.idleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  /** sets speed of motor */
  @Override
  public void setSpeed(double speed) {
    desiredSpeed = speed;
    velocityPID.setReference(speed, ControlType.kVelocity);
  }

  @Override
  public void setP(double p) {
    config.closedLoop.p(p);
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    kP = p;
  }

  @Override
  public void setI(double i) {
    config.closedLoop.i(i);
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    kI = i;
  }

  @Override
  public void setD(double d) {
    config.closedLoop.d(d);
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    kD = d;
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
  public void stop() {
    setVoltage(0.0);
  }

  @Override
  public void setPIDConstants(double p, double i, double d) {
    config.closedLoop.pid(p, i, d);
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    kP = p;
    kI = i;
    kD = d;
  }
}
