package frc.robot.subsystems.elevator;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants;

public class ElevatorIOSparkMax implements ElevatorIO {
  private SparkMax leftMotor;
  // right follows left
  private SparkMax rightMotor;
  private SparkClosedLoopController leftController;
  private AbsoluteEncoder leftEncoder;

  // used to track the target setpoint of the robot
  private double setpoint = 0;

  // Because sparkmax does not have getters for pid, use these variables to keep track of dynamic
  // pid values
  private double kP = ElevatorConstants.ELEVATOR_REAL_PID[0];
  private double kI = ElevatorConstants.ELEVATOR_REAL_PID[1];
  private double kD = ElevatorConstants.ELEVATOR_REAL_PID[2];
  private double kFF = ElevatorConstants.ELEVATOR_REAL_PID[3];

  public ElevatorIOSparkMax() {
    leftMotor = new SparkMax(ElevatorConstants.LEFT_MOTOR_ID, MotorType.kBrushless);
    rightMotor = new SparkMax(ElevatorConstants.RIGHT_MOTOR_ID, MotorType.kBrushless);

    leftController = leftMotor.getClosedLoopController();

    leftEncoder = leftMotor.getAbsoluteEncoder();

    SparkMaxConfig leftConfig = new SparkMaxConfig();
    leftConfig
        .smartCurrentLimit(Constants.NEO_CURRENT_LIMIT)
        .idleMode(ElevatorConstants.MOTOR_DEFAULT_IDLE_MODE)
        .voltageCompensation(12.0)
        .inverted(true);
    leftConfig
        .encoder
        .positionConversionFactor(ElevatorConstants.POSITION_CONVERSION_FACTOR)
        .velocityConversionFactor(ElevatorConstants.POSITION_CONVERSION_FACTOR / 60.0);

    leftConfig.closedLoop.pidf(kP, kI, kD, 0);
    leftConfig.closedLoop.maxMotion.maxVelocity(1).maxAcceleration(1);

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
    inputs.setpointMeters = setpoint;
    inputs.positionMeters = getPosition();
    inputs.velocityMetersPerSec = getVelocity();
    inputs.appliedVoltage = leftMotor.getAppliedOutput() * leftMotor.getBusVoltage();

    inputs.motorCurrent =
        new double[] {leftMotor.getOutputCurrent(), rightMotor.getOutputCurrent()};
    inputs.motorTemperature =
        new double[] {leftMotor.getMotorTemperature(), rightMotor.getMotorTemperature()};
  }

  @Override
  public double getSetpoint() {
    return setpoint;
  }

  @Override
  public double getPosition() {
    // get the absolute position in radians, then convert to meters
    return (leftEncoder.getPosition() + ElevatorConstants.ELEVATOR_OFFSET_METERS);
  }

  @Override
  public void goToSetpoint(double setpoint) {
    this.setpoint = setpoint;

    leftController.setReference(
        setpoint, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0, kFF);
  }

  @Override
  public boolean atSetpoint() {
    // if the difference between setpoint and position is less than the tolerance
    return (Math.abs(getSetpoint() - getPosition()) < ElevatorConstants.SETPOINT_TOLERANCE_METERS);
  }

  @Override
  public double getVelocity() {
    return leftEncoder.getVelocity();
  }

  @Override
  public void setSpeed(double speed) {
    leftMotor.set(speed);
  }

  @Override
  public void setBrakeMode(boolean brakeEnabled) {
    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(brakeEnabled ? IdleMode.kBrake : IdleMode.kCoast);
    updateMotorConfig(config);
  }

  @Override
  public double getP() {
    return kP;
  }

  @Override
  public double getI() {
    return kI;
  }

  @Override
  public double getD() {
    return kD;
  }

  @Override
  public double getFF() {
    return kFF;
  }

  @Override
  public void setP(double kP) {
    this.kP = kP;
    SparkMaxConfig config = new SparkMaxConfig();
    config.closedLoop.p(kP);
    updateMotorConfig(config);
  }

  @Override
  public void setI(double kI) {
    this.kI = kI;
    SparkMaxConfig config = new SparkMaxConfig();
    config.closedLoop.i(kI);
    updateMotorConfig(config);
  }

  @Override
  public void setD(double kD) {
    this.kD = kD;
    SparkMaxConfig config = new SparkMaxConfig();
    config.closedLoop.d(kD);
    updateMotorConfig(config);
  }

  @Override
  public void setFF(double kFF) {
    this.kFF = kFF;
  }
}
