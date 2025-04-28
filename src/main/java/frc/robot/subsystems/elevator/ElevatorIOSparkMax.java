package frc.robot.subsystems.elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class ElevatorIOSparkMax implements ElevatorIO {
  private SparkMax leftMotor;
  // right follows left
  private SparkMax rightMotor;
  private RelativeEncoder leftEncoder;

  // Limit switch used to block elevator if it goes too high
  private DigitalInput limitSwitch;

  // PID controller uses motion profiling to smoothly move setpoint from current position to goal
  // position
  // Feed forward tracks the setpoint's velocity/acceleration to move motors without the need for
  // much PID
  // PID is only used for small corrections
  private ProfiledPIDController pidController =
      new ProfiledPIDController(
          ElevatorConstants.ELEVATOR_REAL_PID[0],
          ElevatorConstants.ELEVATOR_REAL_PID[1],
          ElevatorConstants.ELEVATOR_REAL_PID[2],
          new TrapezoidProfile.Constraints(
              ElevatorConstants.MAX_VELOCITY, ElevatorConstants.MAX_ACCELERATION));

  private ElevatorFeedforward feedforward =
      new ElevatorFeedforward(
          ElevatorConstants.ELEVATOR_REAL_FF[0],
          ElevatorConstants.ELEVATOR_REAL_FF[1],
          ElevatorConstants.ELEVATOR_REAL_FF[2],
          ElevatorConstants.ELEVATOR_REAL_FF[3]);

  // These variables are used to find the acceleration of the PID setpoint
  // (change in velocity / time = avg acceleration)
  double lastSpeed = 0;
  double lastTime = Timer.getFPGATimestamp();

  public ElevatorIOSparkMax() {
    leftMotor = new SparkMax(ElevatorConstants.LEFT_MOTOR_ID, MotorType.kBrushless);
    rightMotor = new SparkMax(ElevatorConstants.RIGHT_MOTOR_ID, MotorType.kBrushless);

    leftEncoder = leftMotor.getEncoder();
    leftEncoder.setPosition(0);

    SparkMaxConfig leftConfig = new SparkMaxConfig();
    leftConfig
        .smartCurrentLimit(Constants.NEO_CURRENT_LIMIT)
        .idleMode(ElevatorConstants.MOTOR_DEFAULT_IDLE_MODE)
        .voltageCompensation(12.0)
        .inverted(false);
    leftConfig
        .encoder
        .positionConversionFactor(ElevatorConstants.POSITION_CONVERSION_FACTOR)
        .velocityConversionFactor(ElevatorConstants.POSITION_CONVERSION_FACTOR / 60.0);

    SparkMaxConfig rightConfig = new SparkMaxConfig();
    rightConfig.apply(leftConfig);
    rightConfig.follow(leftMotor, true);

    // reset safe kResetSafeParameters switches the motor to default paramaters, then adds the
    // changes from the config object
    // persist paramaters saves these changes to the motor memory so it doesn't get cooked during
    // brownouts
    // only use persist in the BEGINNING, not later
    leftMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightMotor.configure(
        rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    pidController.setTolerance(ElevatorConstants.SETPOINT_TOLERANCE_METERS);

    limitSwitch = new DigitalInput(ElevatorConstants.LIMIT_SWITCH_CHANNEL);
  }

  private void updateMotorConfig(SparkMaxConfig config) {
    // DO NOT RESET paramaters becasue we only want to change some paramaters, not all
    // DO NOT PERSIST because this is a temporary change that we don't want to save to memory
    leftMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    rightMotor.configure(
        config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.data = new ElevatorIOData( 
        getSetpoint(),
        getPosition(),
        getVelocity(),
        leftMotor.getAppliedOutput() * leftMotor.getBusVoltage(),
        isLimitSwitchPressed(),
        new double[] {leftMotor.getMotorTemperature(), rightMotor.getMotorTemperature()},
        new double[] {leftMotor.getOutputCurrent(), rightMotor.getOutputCurrent()}
    );
  }

  @Override
  public double getSetpoint() {
    return pidController.getSetpoint().position;
  }

  @Override
  public double getPosition() {
    // get the absolute position in radians, then convert to meters
    return leftEncoder.getPosition();
  }

  @Override
  public void setSetpoint(double setpoint) {
    pidController.setGoal(setpoint);
    pidController.reset(getPosition(), getVelocity());
  }

  @Override
  public void goToSetpoint() {
    double pidOutput = pidController.calculate(getPosition());

    // change in velocity / change in time = acceleration
    // Acceleration is used to calculate feedforward
    double acceleration =
        (pidController.getSetpoint().velocity - lastSpeed) / (Timer.getFPGATimestamp() - lastTime);

    double ffOutput = feedforward.calculate(pidController.getSetpoint().velocity, acceleration);

    setVoltage(MathUtil.clamp(pidOutput + ffOutput, -12, 12));

    lastSpeed = pidController.getSetpoint().velocity;
    lastTime = Timer.getFPGATimestamp();
  }

  @Override
  public boolean atSetpoint() {
    // if the difference between setpoint and position is less than the tolerance
    return pidController.atGoal();
  }

  @Override
  public double getVelocity() {
    return leftEncoder.getVelocity();
  }

  @Override
  public void setVoltage(double voltage) {
    leftMotor.setVoltage(voltage);
    Logger.recordOutput("Elevator/Desired Voltage", voltage);
  }

  @Override
  public void setBrakeMode(boolean brakeEnabled) {
    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(brakeEnabled ? IdleMode.kBrake : IdleMode.kCoast);
    updateMotorConfig(config);
  }

  @Override
  public boolean isLimitSwitchPressed() {
    return !limitSwitch.get();
    // return false;
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
  public void setP(double kP) {
    pidController.setP(kP);
  }

  @Override
  public void setI(double kI) {
    pidController.setI(kI);
  }

  @Override
  public void setD(double kD) {
    pidController.setD(kD);
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
}
