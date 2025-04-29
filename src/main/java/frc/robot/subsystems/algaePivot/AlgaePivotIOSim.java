package frc.robot.subsystems.algaePivot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController; //
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.subsystems.algaePivot.AlgaePivotConstants.AlgaePivotSimConstants;
import org.littletonrobotics.junction.Logger;

public class AlgaePivotIOSim implements AlgaePivotIO {

  // idk what motor we're using for the pivots so check with someone and change this accordingingly
  private final DCMotor m_armGearbox = DCMotor.getNEO(1);

  // Standard classes for controlling our arm
  private final ProfiledPIDController m_controller;
  private ArmFeedforward m_feedforward = new ArmFeedforward(0, 0, 0, 0);

  private double appliedVoltage = 0;

  // These variables are used to find the acceleration of the PID setpoint
  // (change in velocity / change in time = avg acceleration)
  double lastSpeed = 0;
  double lastTime = Timer.getFPGATimestamp();

  // Simulation classes help us simulate what's going on, including gravity.

  private SingleJointedArmSim sim =
      new SingleJointedArmSim(
          m_armGearbox,
          AlgaePivotConstants.AlgaePivotSimConstants.kArmReduction,
          SingleJointedArmSim.estimateMOI(
              AlgaePivotConstants.AlgaePivotSimConstants.kArmLength,
              AlgaePivotConstants.AlgaePivotSimConstants.kArmMass),
          AlgaePivotConstants.AlgaePivotSimConstants.kArmLength,
          AlgaePivotConstants.ALGAE_PIVOT_MIN_ANGLE,
          AlgaePivotConstants.ALGAE_PIVOT_MAX_ANGLE,
          true,
          0.1);

  public AlgaePivotIOSim() {
    m_controller =
        new ProfiledPIDController(
            AlgaePivotConstants.AlgaePivotSimConstants.kPivotSimPID[0],
            AlgaePivotConstants.AlgaePivotSimConstants.kPivotSimPID[1],
            AlgaePivotConstants.AlgaePivotSimConstants.kPivotSimPID[2],
            new TrapezoidProfile.Constraints(2.45, 2.45));

    m_controller.setTolerance(0.1, 0.05);

    m_feedforward =
        new ArmFeedforward(
            AlgaePivotSimConstants.kPivotSimFF[0],
            AlgaePivotSimConstants.kPivotSimFF[1],
            AlgaePivotSimConstants.kPivotSimFF[2],
            AlgaePivotSimConstants.kPivotSimFF[3]);
  }

  @Override
  public void updateInputs(AlgaePivotIOInputs inputs) {
    sim.update(0.02);
    inputs.data =
        new AlgaePivotIOData(
            getAngle(),
            sim.getVelocityRadPerSec(),
            appliedVoltage,
            m_controller.getSetpoint().position,
            false,
            new double[] {sim.getCurrentDrawAmps()},
            null);
  }

  @Override
  public void setVoltage(double motorVolts) {
    sim.setInputVoltage(motorVolts);
    appliedVoltage = motorVolts;
  }

  @Override
  public void setSetpoint(double setpoint) {
    m_controller.setGoal(setpoint);
    m_controller.reset(getAngle(), getAngVelocity());
  }

  @Override
  public void goToSetpoint() {
    double pidOutput = m_controller.calculate(getAngle());

    // change in velocity / change in time = acceleration
    // Acceleration is used to calculate feedforward
    double acceleration =
        (m_controller.getSetpoint().velocity - lastSpeed) / (Timer.getFPGATimestamp() - lastTime);

    Logger.recordOutput("CoralPivot/Acceleration", acceleration);

    double ffOutput =
        m_feedforward.calculate(
            m_controller.getSetpoint().position, m_controller.getSetpoint().velocity, acceleration);

    setVoltage(MathUtil.clamp(pidOutput + ffOutput, -12, 12));

    lastSpeed = m_controller.getSetpoint().velocity;
    lastTime = Timer.getFPGATimestamp();
  }

  @Override
  public double getAngle() {
    return sim.getAngleRads();
  }

  @Override
  public double getAngVelocity() {
    return sim.getVelocityRadPerSec();
  }

  @Override
  public boolean atSetpoint() {
    return m_controller.atGoal();
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
    m_feedforward =
        new ArmFeedforward(kS, m_feedforward.getKg(), m_feedforward.getKv(), m_feedforward.getKa());
  }

  @Override
  public void setkG(double kG) {
    m_feedforward =
        new ArmFeedforward(m_feedforward.getKs(), kG, m_feedforward.getKv(), m_feedforward.getKa());
  }

  @Override
  public void setkV(double kV) {
    m_feedforward =
        new ArmFeedforward(m_feedforward.getKs(), m_feedforward.getKg(), kV, m_feedforward.getKa());
  }

  @Override
  public void setkA(double kA) {
    m_feedforward =
        new ArmFeedforward(m_feedforward.getKs(), m_feedforward.getKg(), m_feedforward.getKv(), kA);
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
    return m_feedforward.getKs();
  }

  @Override
  public double getkG() {
    return m_feedforward.getKg();
  }

  @Override
  public double getkV() {
    return m_feedforward.getKv();
  }

  @Override
  public double getkA() {
    return m_feedforward.getKa();
  }
}
