package frc.robot.subsystems.coralPivot;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class CoralPivotIOSim implements coralPivotIO {

    // idk what motor we're using for the pivots so check with someone and change this accordingingly
    private final DCMotor m_armGearbox = DCMotor.getNEO(1);

    // Standard classes for controlling our arm
    private final ProfiledPIDController m_controller;
    private SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(0, 0);
    private final Encoder m_encoder;

    // Simulation classes help us simulate what's going on, including gravity.

    private SingleJointedArmSim sim = new SingleJointedArmSim(
            m_armGearbox,
            coralPivotConstants.CoralPivotSimConstants.kArmReduction,
            SingleJointedArmSim.estimateMOI(coralPivotConstants.CoralPivotSimConstants.kArmLength, coralPivotConstants.CoralPivotSimConstants.kArmMass),
            coralPivotConstants.CoralPivotSimConstants.kArmLength,
            coralPivotConstants.CoralPivotSimConstants.kMinAngleRads,
            coralPivotConstants.CoralPivotSimConstants.kMaxAngleRads,
            true,
            0.1);

    private final EncoderSim m_encoderSim;

    public CoralPivotIOSim() {
        m_encoder = new Encoder(coralPivotConstants.CoralPivotSimConstants.kEncoderAChannel, coralPivotConstants.CoralPivotSimConstants.kEncoderBChannel);
        m_encoderSim = new EncoderSim(m_encoder);
        m_encoderSim.setDistancePerPulse(coralPivotConstants.CoralPivotSimConstants.kArmEncoderDistPerPulse);
        m_controller = new ProfiledPIDController(coralPivotConstants.CoralPivotSimConstants.kPivotSimPID[0], coralPivotConstants.CoralPivotSimConstants.kPivotSimPID[1], coralPivotConstants.CoralPivotSimConstants.kPivotSimPID[2],
                new TrapezoidProfile.Constraints(2.45, 2.45));
        
        m_controller.setTolerance(0.1, 0.05);
    }

    @Override
    public void updateInputs(CoralPivotIOInputs inputs) {
        sim.update(0.02);
        inputs.angleRads = getAngle();
        inputs.angVelocityRadsPerSec = sim.getVelocityRadPerSec();
        inputs.currentAmps = new double[] { sim.getCurrentDrawAmps() };
        inputs.setpointAngleRads = m_controller.getSetpoint().position;
    }

    @Override
    public void setVoltage(double motorVolts) {
        sim.setInputVoltage(motorVolts);
    }

    @Override
    public void goToSetpoint(double setpoint) {
        m_controller.setGoal(setpoint);
        // With the setpoint value we run PID control like normal
        double pidOutput = m_controller.calculate(getAngle());
        double feedforwardOutput = m_feedforward.calculate(m_controller.getSetpoint().velocity);

        sim.setInputVoltage(feedforwardOutput + pidOutput);
    }

    @Override
    public double getAngle() {
        return sim.getAngleRads();
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
        m_feedforward = new SimpleMotorFeedforward(kS, m_feedforward.getKv());
    }

    @Override
    public void setkV(double kV) {
        m_feedforward = new SimpleMotorFeedforward(m_feedforward.getKs(), kV);
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
    public double getkV() {
        return m_feedforward.getKv();
    }

}