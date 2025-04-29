package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
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
  private final ElevatorFeedforward feedforward;

  private double appliedVoltage = 0;

  // These variables are used to find the acceleration of the PID setpoint (change in velocity /
  // time = avg acceleration)
  double lastSpeed = 0;
  double lastTime = Timer.getFPGATimestamp();

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
          ElevatorSimConstants.ELEVATOR_STARTING_HEIGHT);

  public ElevatorIOSim() {
    m_controller =
        new ProfiledPIDController(
            ElevatorSimConstants.ELEVATOR_SIM_PID[0],
            ElevatorSimConstants.ELEVATOR_SIM_PID[1],
            ElevatorSimConstants.ELEVATOR_SIM_PID[2],
            new TrapezoidProfile.Constraints(0.6, 5));

    m_controller.setTolerance(0.1, 0.05);

    feedforward =
        new ElevatorFeedforward(
            ElevatorSimConstants.ELEVATOR_SIM_FF[0],
            ElevatorSimConstants.ELEVATOR_SIM_FF[1],
            ElevatorSimConstants.ELEVATOR_SIM_FF[2],
            ElevatorSimConstants.ELEVATOR_SIM_FF[3]);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    sim.update(0.02);
    inputs.data =
        new ElevatorIOData(
            m_controller.getSetpoint().position,
            sim.getPositionMeters(),
            sim.getVelocityMetersPerSecond(),
            appliedVoltage,
            false,
            new double[] {},
            new double[] {sim.getCurrentDrawAmps()});
  }

  @Override
  public double getSetpoint() {
    return m_controller.getSetpoint().position;
  }

  @Override
  public void setSetpoint(double setpoint) {
    m_controller.setGoal(setpoint);
    m_controller.reset(getPosition(), getVelocity());
  }

  @Override
  public void goToSetpoint() {
    double pidOutput = m_controller.calculate(getPosition());

    // change in velocity / change in time = acceleration
    // Acceleration is used to calculate feedforward
    double acceleration =
        (m_controller.getSetpoint().velocity - lastSpeed) / (Timer.getFPGATimestamp() - lastTime);

    double ffOutput = feedforward.calculate(m_controller.getSetpoint().velocity, acceleration);

    setVoltage(MathUtil.clamp(pidOutput + ffOutput, -12, 12));

    lastSpeed = m_controller.getSetpoint().velocity;
    lastTime = Timer.getFPGATimestamp();
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
}
