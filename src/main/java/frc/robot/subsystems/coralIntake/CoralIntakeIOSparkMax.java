package frc.robot.subsystems.coralIntake;

import static frc.robot.Constants.NEO_CURRENT_LIMIT;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

/** Need to import Constants files/classes */
//

public class CoralIntakeIOSparkMax implements CoralIntakeIO {

  private SparkMax motor;
  private RelativeEncoder encoder;

  public CoralIntakeIOSparkMax() {
    /** ID needs to be assigned from constants */
    // setPIDConstants(kGroundIntakeP, kGroundIntakeI, kGroundIntakeD);
    motor = new SparkMax(CoralIntakeConstants.CORAL_INTAKE_MOTOR_ID, MotorType.kBrushless);
    encoder = motor.getEncoder();

    SparkMaxConfig config = new SparkMaxConfig();
    config
        .idleMode(com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kCoast)
        .voltageCompensation(12)
        .smartCurrentLimit(NEO_CURRENT_LIMIT)
        .inverted(true);

    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  /** updates inputs from robot */
  @Override
  public void updateInputs(CoralIntakeIOInputs inputs) {
    inputs.appliedVoltage = motor.getAppliedOutput() * motor.getBusVoltage();
    // duty cycle (scale from 0 to 1) * actual voltage given to the motor
    inputs.currentAmps = new double[] {motor.getOutputCurrent()};
    inputs.tempCelcius = new double[] {motor.getMotorTemperature()};
    inputs.velocityRadsPerSec = encoder.getVelocity();
  }

  /** sets voltage to run motor if necessary */
  @Override
  public void setVoltage(double voltage) {
    motor.setVoltage(voltage);
  }

  /** sets brake mode to stop */
  @Override
  public void setBrake(boolean brake) {
    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
    motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }
}
