// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

import static frc.robot.subsystems.drive.DriveConstants.kFrontLeftDrivingCanId;
import static frc.robot.subsystems.drive.DriveConstants.kFrontLeftTurningCanId;
import static frc.robot.subsystems.drive.DriveConstants.kFrontRightDrivingCanId;
import static frc.robot.subsystems.drive.DriveConstants.kFrontRightTurningCanId;
import static frc.robot.subsystems.drive.DriveConstants.kRearLeftDrivingCanId;
import static frc.robot.subsystems.drive.DriveConstants.kRearLeftTurningCanId;
import static frc.robot.subsystems.drive.DriveConstants.kRearRightDrivingCanId;
import static frc.robot.subsystems.drive.DriveConstants.kRearRightTurningCanId;
import static frc.robot.subsystems.drive.ModuleConstants.kDrivingEncoderPositionFactor;
import static frc.robot.subsystems.drive.ModuleConstants.kDrivingEncoderVelocityFactor;
import static frc.robot.subsystems.drive.ModuleConstants.kDrivingMaxOutput;
import static frc.robot.subsystems.drive.ModuleConstants.kDrivingMinOutput;
import static frc.robot.subsystems.drive.ModuleConstants.kDrivingMotorCurrentLimit;
import static frc.robot.subsystems.drive.ModuleConstants.kDrivingMotorIdleMode;
import static frc.robot.subsystems.drive.ModuleConstants.kTurningEncoderPositionFactor;
import static frc.robot.subsystems.drive.ModuleConstants.kTurningEncoderPositionPIDMaxInput;
import static frc.robot.subsystems.drive.ModuleConstants.kTurningEncoderPositionPIDMinInput;
import static frc.robot.subsystems.drive.ModuleConstants.kTurningEncoderVelocityFactor;
import static frc.robot.subsystems.drive.ModuleConstants.kTurningMaxOutput;
import static frc.robot.subsystems.drive.ModuleConstants.kTurningMinOutput;
import static frc.robot.subsystems.drive.ModuleConstants.kTurningMotorCurrentLimit;
import static frc.robot.subsystems.drive.ModuleConstants.kTurningMotorIdleMode;
import static frc.robot.subsystems.drive.ModuleConstants.kWheelDiameterMeters;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import java.util.Queue;

/**
 * Module IO implementation for SparkMax drive motor controller, SparkMax turn motor controller (NEO
 * or NEO 550), and analog absolute encoder connected to the RIO
 *
 * <p>NOTE: This implementation should be used as a starting point and adapted to different hardware
 * configurations (e.g. If using a CANcoder, copy from "ModuleIOTalonFX")
 *
 * <p>To calibrate the absolute encoder offsets, point the modules straight (such that forward
 * motion on the drive motor will propel the robot forward) and copy the reported values from the
 * absolute encoders using AdvantageScope. These values are logged under
 * "/Drive/ModuleX/TurnAbsolutePositionRad"
 */
public class ModuleIOSparkMax implements ModuleIO {

  private final SparkMax driveSparkMax;
  private final SparkMax turnSparkMax;

  private final SparkMaxConfig driveConfig;
  private final SparkMaxConfig turnConfig;

  private final SparkClosedLoopController drivePIDController;
  private final SparkClosedLoopController turnPIDController;

  private final RelativeEncoder driveEncoder;
  private final AbsoluteEncoder turnAbsoluteEncoder;

  private final Queue<Double> timestampQueue;
  private final Queue<Double> drivePositionQueue;
  private final Queue<Double> turnPositionQueue;

  private final double absoluteEncoderOffset;

  public ModuleIOSparkMax(int index) {
    switch (index) {
      case 0: // Front Left
        driveSparkMax = new SparkMax(kFrontLeftDrivingCanId, MotorType.kBrushless);
        turnSparkMax = new SparkMax(kFrontLeftTurningCanId, MotorType.kBrushless);
        absoluteEncoderOffset = DriveConstants.kFrontLeftChassisAngularOffset; // MUST BE CALIBRATED
        break;
      case 1: // Front Right
        driveSparkMax = new SparkMax(kFrontRightDrivingCanId, MotorType.kBrushless);
        turnSparkMax = new SparkMax(kFrontRightTurningCanId, MotorType.kBrushless);
        absoluteEncoderOffset =
            DriveConstants.kFrontRightChassisAngularOffset; // MUST BE CALIBRATED
        break;
      case 2: // Back Left
        driveSparkMax = new SparkMax(kRearLeftDrivingCanId, MotorType.kBrushless);
        turnSparkMax = new SparkMax(kRearLeftTurningCanId, MotorType.kBrushless);
        absoluteEncoderOffset = DriveConstants.kBackLeftChassisAngularOffset; // MUST BE CALIBRATED
        break;
      case 3: // Back Right
        driveSparkMax = new SparkMax(kRearRightDrivingCanId, MotorType.kBrushless);
        turnSparkMax = new SparkMax(kRearRightTurningCanId, MotorType.kBrushless);
        absoluteEncoderOffset = DriveConstants.kBackRightChassisAngularOffset; // MUST BE CALIBRATED
        break;
      default:
        throw new RuntimeException("Invalid module index");
    }

    driveConfig = new SparkMaxConfig();
    turnConfig = new SparkMaxConfig();

    // Configure motor current limit, voltage compensation, and brake modes
    driveConfig
        .voltageCompensation(12.0)
        .smartCurrentLimit(kDrivingMotorCurrentLimit)
        .idleMode(kDrivingMotorIdleMode);

    turnConfig
        .voltageCompensation(12)
        .smartCurrentLimit(kTurningMotorCurrentLimit)
        .idleMode(kTurningMotorIdleMode);

    driveSparkMax.setCANTimeout(250);
    turnSparkMax.setCANTimeout(250);

    // Apply position and velocity conversion factors for the turning encoder. We
    // want these in radians and radians per second to use with WPILib's swerve
    // APIs.
    turnConfig
        .absoluteEncoder
        .positionConversionFactor(kTurningEncoderPositionFactor)
        .velocityConversionFactor(kTurningEncoderVelocityFactor)
        .inverted(ModuleConstants.kTurningEncoderInverted);

    // Configure drive encoder
    driveConfig
        .encoder
        .positionConversionFactor(kDrivingEncoderPositionFactor)
        .velocityConversionFactor(kDrivingEncoderVelocityFactor);

    // Configure Drive PID Controller
    driveConfig
        .closedLoop
        .outputRange(kDrivingMinOutput, kDrivingMaxOutput)
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder);

    // Configure Turn PID Controller
    // Enable PID wrap around for the turning motor. This will allow the PID
    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
    // to 10 degrees will go through 0 rather than the other direction which is a
    // longer route.
    turnConfig
        .closedLoop
        .positionWrappingEnabled(true)
        .positionWrappingMinInput(kTurningEncoderPositionPIDMinInput)
        .positionWrappingMaxInput(kTurningEncoderPositionPIDMaxInput)
        .outputRange(kTurningMinOutput, kTurningMaxOutput)
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder);

    driveSparkMax.setCANTimeout(0);
    turnSparkMax.setCANTimeout(0);

    // Log things?
    driveConfig.signals.primaryEncoderPositionPeriodMs((int) (1000.0 / Module.ODOMETRY_FREQUENCY));
    turnConfig.signals.primaryEncoderPositionPeriodMs((int) (1000.0 / Module.ODOMETRY_FREQUENCY));

    // Save our settings
    driveSparkMax.configure(
        driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    turnSparkMax.configure(
        turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    driveEncoder = driveSparkMax.getEncoder();
    turnAbsoluteEncoder = turnSparkMax.getAbsoluteEncoder();

    drivePIDController = driveSparkMax.getClosedLoopController();
    turnPIDController = turnSparkMax.getClosedLoopController();

    timestampQueue = SparkMaxOdometryThread.getInstance().makeTimestampQueue();
    drivePositionQueue =
        SparkMaxOdometryThread.getInstance().registerSignal(driveEncoder::getPosition);
    turnPositionQueue =
        SparkMaxOdometryThread.getInstance().registerSignal(turnAbsoluteEncoder::getPosition);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    inputs.drivePositionRad = driveEncoder.getPosition() / (kWheelDiameterMeters / 2);
    inputs.drivePositionMeters = driveEncoder.getPosition();
    inputs.driveVelocityMeterPerSec = driveEncoder.getVelocity();
    inputs.driveVelocityRadPerSec = driveEncoder.getVelocity() / (kWheelDiameterMeters / 2);
    inputs.driveAppliedVolts = driveSparkMax.getAppliedOutput() * driveSparkMax.getBusVoltage();
    inputs.driveCurrentAmps = new double[] {driveSparkMax.getOutputCurrent()};

    inputs.turnAbsolutePosition = getTurnPosition();
    inputs.turnPosition = getTurnPosition();
    inputs.turnVelocityRadPerSec = turnAbsoluteEncoder.getVelocity();
    inputs.turnAppliedVolts = turnSparkMax.getAppliedOutput() * turnSparkMax.getBusVoltage();
    inputs.turnCurrentAmps = new double[] {turnSparkMax.getOutputCurrent()};

    inputs.odometryTimestamps =
        timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryDrivePositionsRad =
        drivePositionQueue.stream()
            .mapToDouble((Double value) -> Units.rotationsToRadians(value))
            .toArray();
    inputs.odometryTurnPositions =
        turnPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromRotations(value))
            .toArray(Rotation2d[]::new);
    timestampQueue.clear();
    drivePositionQueue.clear();
    turnPositionQueue.clear();
  }

  @Override
  public void setDrivePIDFF(double p, double i, double d, double ff) {
    driveConfig.closedLoop.pidf(p, i, d, ff);
    driveSparkMax.configure(
        driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void setTurnPIDFF(double p, double i, double d, double ff) {
    turnConfig.closedLoop.pidf(p, i, d, ff);
    turnSparkMax.configure(
        turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void setDriveVelocity(double velocityRadPerSec) {
    drivePIDController.setReference(velocityRadPerSec, SparkMax.ControlType.kVelocity);
  }

  @Override
  public void setTurnPosition(double angle) {
    turnPIDController.setReference(angle + absoluteEncoderOffset, SparkMax.ControlType.kPosition);
  }

  public Rotation2d getTurnPosition() {
    double angle = turnAbsoluteEncoder.getPosition() - absoluteEncoderOffset;

    return Rotation2d.fromRadians(angle);
  }

  @Override
  public void setDriveVoltage(double volts) {
    driveSparkMax.setVoltage(volts);
  }

  @Override
  public void setTurnVoltage(double volts) {
    turnSparkMax.setVoltage(volts);
  }

  @Override
  public void setDriveBrakeMode(boolean enable) {
    driveConfig.idleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
    driveSparkMax.configure(
        driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void setTurnBrakeMode(boolean enable) {
    driveConfig.idleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
    driveSparkMax.configure(
        driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public double getTurnPositionError(double angle) {
    return Math.abs(turnAbsoluteEncoder.getPosition() - angle);
  }

  @Override
  public double getAbsoluteEncoderOffset() {
    return absoluteEncoderOffset;
  }
}
