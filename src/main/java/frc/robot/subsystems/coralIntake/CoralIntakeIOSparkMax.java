package frc.robot.subsystems.coralIntake;

import static frc.robot.Constants.NEO_CURRENT_LIMIT;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj.DigitalInput;
import org.littletonrobotics.junction.Logger;

/** Need to import Constants files/classes */
//

public class CoralIntakeIOSparkMax implements CoralIntakeIO {

  private SparkFlex motor;
  private RelativeEncoder encoder;
  private DigitalInput breakBeam;

  public CoralIntakeIOSparkMax() {
    /** ID needs to be assigned from constants */
    // setPIDConstants(kGroundIntakeP, kGroundIntakeI, kGroundIntakeD);
    motor = new SparkFlex(CoralIntakeConstants.CORAL_INTAKE_MOTOR_ID, MotorType.kBrushless);
    encoder = motor.getEncoder();

    SparkFlexConfig config = new SparkFlexConfig();
    config
        .idleMode(IdleMode.kBrake)
        .voltageCompensation(12)
        .smartCurrentLimit(NEO_CURRENT_LIMIT)
        .inverted(true);

    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    breakBeam = new DigitalInput(CoralIntakeConstants.BREAK_BEAM_CHANNEL);
  }

  /** updates inputs from robot */
  @Override
  public void updateInputs(CoralIntakeIOInputs inputs) {
    inputs.data =
        new CoralIntakeIOData(
            encoder.getVelocity(),
            motor.getAppliedOutput() * motor.getBusVoltage(),
            new double[] {motor.getOutputCurrent()},
            new double[] {motor.getMotorTemperature()},
            isBreakBeamBroken());
  }

  /** sets voltage to run motor if necessary */
  @Override
  public void setVoltage(double voltage) {
    motor.setVoltage(voltage);
    Logger.recordOutput("CoralIntake/Desired Voltage", voltage);
  }

  /** sets brake mode to stop */
  @Override
  public void setBrake(boolean brake) {
    SparkFlexConfig config = new SparkFlexConfig();
    config.idleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
    motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  /** Returns true if break beam is broken, false otherwise */
  @Override
  public boolean isBreakBeamBroken() {
    return breakBeam.get();
  }

  @Override
  public double getVelocity() {
    return encoder.getVelocity();
  }
}
