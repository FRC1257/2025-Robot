package frc.robot.subsystems.algaeIntake;

import static frc.robot.Constants.NEO_CURRENT_LIMIT;
import static frc.robot.subsystems.algaeIntake.AlgaeIntakeConstants.AlgaeIntakePhysicalConstants;
import static frc.robot.subsystems.algaeIntake.AlgaeIntakeConstants.AlgaeIntakePhysicalConstants.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants.ElectricalLayout;

/** Need to import Constants files/classes */
//

public class AlgaeIntakeIOSparkMax implements AlgaeIntakeIO {

  private SparkMax motor;
  private RelativeEncoder encoder;
  private SparkClosedLoopController velocityPID;

  private SparkMaxConfig config;

  private double desiredSpeed;

  private double kP, kI, kD;

  public AlgaeIntakeIOSparkMax() {
    /** ID needs to be assigned from constants */
    // setPIDConstants(kGroundIntakeP, kGroundIntakeI, kGroundIntakeD);
    motor = new SparkMax(ElectricalLayout.ALGAE_INTAKE_MOTOR, SparkMax.MotorType.kBrushless);

    config
        .idleMode(com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kCoast)
        .smartCurrentLimit(NEO_CURRENT_LIMIT)
        .inverted(true);

    encoder = motor.getEncoder();

    velocityPID = motor.getClosedLoopController();

    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    kP = AlgaeIntakePhysicalConstants.kAlgaeIntakeP;
    kI = AlgaeIntakePhysicalConstants.kAlgaeIntakeI;
    kD = AlgaeIntakePhysicalConstants.kAlgaeIntakeD;
  }

  /** updates inputs from robot */
  @Override
  public void updateInputs(AlgaeIntakeIOInputs inputs) {
    inputs.appliedVoltage = motor.getAppliedOutput() * motor.getBusVoltage();
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
    config.idleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }
}
