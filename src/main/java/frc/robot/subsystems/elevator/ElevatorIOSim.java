package frc.robot.subsystems.elevator;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;

public class ElevatorIOSim implements ElevatorIO {
  // from here
  // https://github.com/wpilibsuite/allwpilib/blob/main/wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/armsimulation/subsystems/Arm.java
  // The P gain for the PID controller that drives this arm.

  // The arm gearbox represents a gearbox containing two Vex 775pro motors.
  private final DCMotor m_elevatorGearBox = DCMotor.getNEO(2);

  // Standard classes for controlling our arm
  private final ProfiledPIDController m_controller;
  private SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(0, 0);
  private final Encoder m_encoder;

  // Simulation classes help us simulate what's going on, including gravity.
  // This arm sim represents an arm that can travel from -75 degrees (rotated down
  // front)
  // to 255 degrees (rotated down in the back).

  private ElevatorSim sim =
      new ElevatorSim(
          m_elevatorGearBox,
          ElevatorConstants.GEAR_RATIO,
          ElevatorConstants.ELEVATOR_MASS_KG,
          ElevatorConstants.MOTOR_RADIUS_METERS,
          ElevatorConstants.ELEVATOR_MIN_HEIGHT + ElevatorConstants.ELEVATOR_OFFSET_METERS,
          ElevatorConstants.ELEVATOR_MAX_HEIGHT + ElevatorConstants.ELEVATOR_OFFSET_METERS,
          true, // change this to true later
          ElevatorConstants.ELEVATOR_STARTING_HEIGHT);

  private final EncoderSim m_encoderSim;

  public ElevatorIOSim() {
    m_encoder =
        new Encoder(
            ElevatorConstants.ElevatorSimConstants.kEncoderAChannel,
            ElevatorConstants.ElevatorSimConstants.kEncoderBChannel);
    m_encoderSim = new EncoderSim(m_encoder);
    m_encoderSim.setDistancePerPulse(ElevatorConstants.ElevatorSimConstants.ENCODER_DIST_PER_PULSE);
    m_controller =
        new ProfiledPIDController(
            ElevatorConstants.ElevatorSimConstants.kElevatorSimPID[0],
            ElevatorConstants.ElevatorSimConstants.kElevatorSimPID[1],
            ElevatorConstants.ElevatorSimConstants.kElevatorSimPID[2],
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
    inputs.appliedVoltage = 0;
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
    double feedforwardOutput = m_feedforward.calculate(m_controller.getSetpoint().velocity);

    sim.setInputVoltage(feedforwardOutput + pidOutput);
  }

  @Override
  public double getPosition() {
    return m_encoderSim.getDistance();
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
  public void setVelocity(double velocity) {
    sim.setInputVoltage(velocity * 12);
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
}
