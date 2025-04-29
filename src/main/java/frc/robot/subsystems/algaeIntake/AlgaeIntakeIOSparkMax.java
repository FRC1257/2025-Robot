package frc.robot.subsystems.algaeIntake;

import static frc.robot.Constants.NEO_VORTEX_CURRENT_LIMIT;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import org.littletonrobotics.junction.Logger;

/** Need to import Constants files/classes */
//

public class AlgaeIntakeIOSparkMax implements AlgaeIntakeIO {

  private SparkFlex motor;
  private RelativeEncoder encoder;

  private SparkFlexConfig config = new SparkFlexConfig();

  public AlgaeIntakeIOSparkMax() {
    /** ID needs to be assigned from constants */
    // setPIDConstants(kGroundIntakeP, kGroundIntakeI, kGroundIntakeD);
    motor = new SparkFlex(AlgaeIntakeConstants.ALGAE_INTAKE_MOTOR_ID, MotorType.kBrushless);

    config
        .idleMode(IdleMode.kBrake)
        .voltageCompensation(12)
        .smartCurrentLimit(NEO_VORTEX_CURRENT_LIMIT)
        .inverted(true);

    encoder = motor.getEncoder();

    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  /** updates inputs from robot */
  @Override
  public void updateInputs(AlgaeIntakeIOInputs inputs) {
    inputs.data =
        new AlgaeIntakeIOData(
            motor.getAppliedOutput() * motor.getBusVoltage(),
            encoder.getVelocity(),
            new double[] {motor.getOutputCurrent()},
            new double[] {motor.getMotorTemperature()});
  }

  /** sets voltage to run motor if necessary */
  @Override
  public void setVoltage(double voltage) {
    motor.setVoltage(voltage);
    Logger.recordOutput("AlgaeIntake/Desired Voltage", voltage);
  }

  /** sets brake mode to stop */
  @Override
  public void setBrake(boolean brake) {
    config.idleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
    motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }
}
