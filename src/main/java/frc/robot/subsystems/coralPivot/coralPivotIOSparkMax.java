// Made just to change spark max because commit didn't work on a different laptop and I don't want
// merging issues

package frc.robot.subsystems.coralPivot;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class CoralPivotIOSparkMax implements CoralPivotIO {
  // Motor and Encoders
  private SparkMax pivotMotor;
  private final ProfiledPIDController pidController;
  private ArmFeedforward feedforward = new ArmFeedforward(0, 0, 0, 0);

  private AbsoluteEncoder absoluteEncoder;

  private double setpoint = 0;

  public CoralPivotIOSparkMax() {
    pivotMotor = new SparkMax(CoralPivotConstants.CORAL_PIVOT_ID, MotorType.kBrushless);

    SparkMaxConfig config = new SparkMaxConfig();

    setBrake(true);

    config.voltageCompensation(12.0).smartCurrentLimit(Constants.NEO_CURRENT_LIMIT);

    absoluteEncoder = pivotMotor.getAbsoluteEncoder();

    config
        .absoluteEncoder
        .positionConversionFactor(2 * Constants.PI * CoralPivotConstants.POSITION_CONVERSION_FACTOR)
        .velocityConversionFactor(
            2 * Constants.PI * CoralPivotConstants.POSITION_CONVERSION_FACTOR / 60.0);

    // absoluteEncoder.reset();
    // make sure the pivot starts at the bottom position every time
    // absoluteEncoder.reset();

    pidController =
        new ProfiledPIDController(
            CoralPivotConstants.CORAL_PIVOT_PID_REAL[0],
            CoralPivotConstants.CORAL_PIVOT_PID_REAL[1],
            CoralPivotConstants.CORAL_PIVOT_PID_REAL[2],
            new TrapezoidProfile.Constraints(2.45, 2.45));

    pidController.setTolerance(
        CoralPivotConstants.CORAL_PIVOT_PID_TOLERANCE,
        CoralPivotConstants.CORAL_PIVOT_PID_VELOCITY_TOLERANCE);

    // 0 position for absolute encoder is at 0.2585 rad, so subtract that value from everything

    pivotMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    configurePID();
    configureFeedForward();

    Logger.recordOutput("Absolute Encoder Starting Position: ", absoluteEncoder.getPosition());
  }

  private void configurePID() {
    // pidController.setOutputRange(CoralPivotConstants.CORAL_PIVOT_MIN_ANGLE,
    // CoralPivotConstants.CORAL_PIVOT_MAX_ANGLE);
    pidController.setP(CoralPivotConstants.CORAL_PIVOT_PID_REAL[0]);
    pidController.setI(CoralPivotConstants.CORAL_PIVOT_PID_REAL[1]);
    pidController.setD(CoralPivotConstants.CORAL_PIVOT_PID_REAL[2]);
  }

  private void configureFeedForward() {
    setkS(CoralPivotConstants.CORAL_PIVOT_FEEDFORWARD_REAL[0]);
    setkG(CoralPivotConstants.CORAL_PIVOT_FEEDFORWARD_REAL[1]);
    setkV(CoralPivotConstants.CORAL_PIVOT_FEEDFORWARD_REAL[2]);
    setkA(CoralPivotConstants.CORAL_PIVOT_FEEDFORWARD_REAL[3]);
  }

  /** Updates the set of loggable inputs. */
  @Override
  public void updateInputs(CoralPivotIOInputs inputs) {
    inputs.angleRads = getAngle();
    Logger.recordOutput("CoralPivot/Absolute", absoluteEncoder.getPosition());
    inputs.angVelocityRadsPerSec = absoluteEncoder.getVelocity();
    inputs.appliedVolts = pivotMotor.getAppliedOutput() * pivotMotor.getBusVoltage();
    inputs.currentAmps = new double[] {pivotMotor.getOutputCurrent()};
    inputs.tempCelsius = new double[] {pivotMotor.getMotorTemperature()};
    inputs.setpointAngleRads = setpoint;
  }

  /** Run open loop at the specified voltage. */
  @Override
  public void setVoltage(double motorVolts) {
    Logger.recordOutput("CoralPivot/AppliedVolts", motorVolts);
    pivotMotor.setVoltage(motorVolts);
  }

  /** Returns the current distance measurement. */
  @Override
  public double getAngle() {
    return -absoluteEncoder.getPosition() + CoralPivotConstants.CORAL_PIVOT_OFFSET;
  }

  /** Go to Setpoint */
  @Override
  public void goToSetpoint(double setpoint) {
    pidController.setGoal(setpoint);
    // With the setpoint value we run PID control like normal
    double pidOutput = MathUtil.clamp(pidController.calculate(getAngle()), -3, 3);
    double feedforwardOutput =
        feedforward.calculate(getAngle(), pidController.getSetpoint().velocity);

    Logger.recordOutput("CoralPivot/FeedforwardOutput", feedforwardOutput);
    Logger.recordOutput("CoralPivot/PIDOutput", pidOutput);

    Logger.recordOutput("CoralPivot/VelocityError", pidController.getVelocityError());

    setVoltage(MathUtil.clamp(pidOutput + feedforwardOutput, -4, 4));
  }

  @Override
  public void holdSetpoint(double setpoint) {
    goToSetpoint(setpoint);
  }

  @Override
  public void setBrake(boolean brake) {
    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
    pivotMotor.configure(
        config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public boolean atSetpoint() {
    return Math.abs(getAngle() - setpoint) < CoralPivotConstants.CORAL_PIVOT_PID_TOLERANCE;
  }

  @Override
  public void setP(double p) {
    pidController.setP(p);
  }

  @Override
  public void setI(double i) {
    pidController.setI(i);
  }

  @Override
  public void setD(double d) {
    pidController.setD(d);
  }

  @Override
  public void setFF(double ff) {
    // pidController.setFF(ff);
  }

  @Override
  public void setkS(double kS) {
    feedforward =
        new ArmFeedforward(kS, feedforward.getKg(), feedforward.getKv(), feedforward.getKa());
  }

  @Override
  public void setkG(double kG) {
    feedforward =
        new ArmFeedforward(feedforward.getKs(), kG, feedforward.getKv(), feedforward.getKa());
  }

  @Override
  public void setkV(double kV) {
    feedforward =
        new ArmFeedforward(feedforward.getKs(), feedforward.getKg(), kV, feedforward.getKa());
  }

  @Override
  public void setkA(double kA) {
    feedforward =
        new ArmFeedforward(feedforward.getKs(), feedforward.getKg(), feedforward.getKv(), kA);
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

  @Override
  public double getP() {
    return pidController.getP();
  }

  @Override
  public double getI() {
    return pidController.getI();
  }

  @Override
  public double getD() {
    return pidController.getD();
  }

  @Override
  public double getFF() {
    // return pidController.getFF();
    return 0;
  }
}
