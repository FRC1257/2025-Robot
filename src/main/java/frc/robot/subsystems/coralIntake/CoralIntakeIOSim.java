package frc.robot.subsystems.coralIntake;

import static frc.robot.subsystems.coralIntake.CoralIntakeConstants.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class CoralIntakeIOSim implements CoralIntakeIO {
  private final FlywheelSim sim =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(
              DCMotor.getNEO(1), kMomentOfInertia, kCoralIntakeGearing),
          DCMotor.getNEO(1));

  private double appliedVoltage = 0.0;

  public CoralIntakeIOSim() {}

  @Override
  public void updateInputs(CoralIntakeIOInputs inputs) {
    sim.update(0.02);
    inputs.data =
        new CoralIntakeIOData(
            sim.getAngularVelocityRadPerSec(),
            appliedVoltage,
            new double[] {sim.getCurrentDrawAmps()},
            new double[] {60},
            false);
  }

  @Override
  public void setVoltage(double volts) {
    appliedVoltage = volts;
    sim.setInputVoltage(volts);
  }

  @Override
  public double getVelocity() {
    return sim.getAngularVelocityRadPerSec();
  }
}
