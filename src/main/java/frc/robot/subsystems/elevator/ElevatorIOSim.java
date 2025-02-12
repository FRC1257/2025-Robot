package frc.robot.subsystems.elevator;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorSimConstants;

public class ElevatorIOSim implements ElevatorIO {
  // from here
  // https://github.com/wpilibsuite/allwpilib/blob/main/wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/armsimulation/subsystems/Arm.java
  // The P gain for the PID controller that drives this arm.

  // The arm gearbox represents a gearbox containing two Vex 775pro motors.
  private final DCMotor m_elevatorGearBox = DCMotor.getNEO(2);

  // Standard classes for controlling our arm
  private final ProfiledPIDController m_controller;
  private double ffOutput = ElevatorSimConstants.ELEVATOR_SIM_PID[3];

  private double appliedVoltage = 0;

  // Simulation classes help us simulate what's going on, including gravity.
  // This arm sim represents an arm that can travel from -75 degrees (rotated down
  // front)
  // to 255 degrees (rotated down in the back).

  private ElevatorSim sim =
      new ElevatorSim(
          m_elevatorGearBox,
          ElevatorSimConstants.GEAR_RATIO_SIM,
          ElevatorConstants.ELEVATOR_MASS_KG,
          ElevatorConstants.DRUM_RADIUS_METERS,
          ElevatorConstants.ELEVATOR_MIN_HEIGHT,
          ElevatorConstants.ELEVATOR_MAX_HEIGHT,
          true, // change this to true later
          ElevatorConstants.ELEVATOR_STARTING_HEIGHT);

  public ElevatorIOSim() {
    m_controller =
        new ProfiledPIDController(
            ElevatorSimConstants.ELEVATOR_SIM_PID[0],
            ElevatorSimConstants.ELEVATOR_SIM_PID[1],
            ElevatorSimConstants.ELEVATOR_SIM_PID[2],
            new TrapezoidProfile.Constraints(2.45, 2.45));

    m_controller.setTolerance(0.1, 0.05);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    sim.update(0.02);
    inputs.positionMeters = getPosition();
    inputs.velocityMetersPerSec = sim.getVelocityMetersPerSecond();
    inputs.motorCurrent = new double[] {sim.getCurrentDrawAmps()};
    inputs.setpointMeters = m_controller.getSetpoint().position;
    inputs.appliedVoltage = appliedVoltage;
  }

  @Override
  public double getSetpoint() {
    return m_controller.getSetpoint().position;
  }

  @Override
  public void goToSetpoint(double setpoint) {
    m_controller.setGoal(setpoint);
    // With the setpoint value we run PID control like normal
    double pidOutput = m_controller.calculate(getPosition());

    setVoltage(ffOutput + pidOutput);
  }

  @Override
  public double getPosition() {
    return sim.getPositionMeters();
  }

  @Override
  public boolean atSetpoint() {
    return m_controller.atGoal();
  }

  @Override
  public double getVelocity() {
    return sim.getVelocityMetersPerSecond();
  }

  @Override
  public void setVoltage(double voltage) {
    appliedVoltage = voltage;
    sim.setInputVoltage(voltage);
  }

  @Override
  public void setP(double p) {
    m_controller.setP(p);
  }

  @Override
  public void setI(double i) {
    m_controller.setI(i);
  }

  @Override
  public void setD(double d) {
    m_controller.setD(d);
  }

  @Override
  public void setFF(double ff) {
    this.ffOutput = ff;
  }

  @Override
  public double getP() {
    return m_controller.getP();
  }

  @Override
  public double getI() {
    return m_controller.getI();
  }

  @Override
  public double getD() {
    return m_controller.getD();
  }

  @Override
  public double getFF() {
    return ffOutput;
  }
}
