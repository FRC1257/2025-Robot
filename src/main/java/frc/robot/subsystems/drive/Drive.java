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

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.useVision;
import static frc.robot.subsystems.drive.DriveConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.FieldConstants;
import frc.robot.commands.AlignToPose;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOInputsAutoLogged;
import frc.robot.util.autonomous.CustomAutoChooser.ReefLevels;
import frc.robot.util.autonomous.CustomAutoChooser.ReefPositions;
import frc.robot.util.autonomous.LocalADStarAK;
import frc.robot.util.drive.AllianceFlipUtil;
import java.util.List;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class Drive extends SubsystemBase {
  // private static final double DRIVE_BASE_RADIUS = Math.hypot(kTrackWidthX / 2.0, kTrackWidthY /
  // 2.0);
  public static final double DRIVE_BASE_RADIUS = Math.hypot(kTrackWidthX / 2.0, kTrackWidthY / 2.0);
  public static final double MAX_ANGULAR_SPEED = kMaxSpeedMetersPerSecond / DRIVE_BASE_RADIUS;

  private RobotConfig config;

  static final Lock odometryLock = new ReentrantLock();
  private final GyroIO gyroIO;
  private GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR
  // private final SysIdRoutine sysId;

  private final VisionIO visionIO;
  private final VisionIOInputsAutoLogged visionInputs = new VisionIOInputsAutoLogged();

  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());
  private Rotation2d rawGyroRotation = new Rotation2d();
  private SwerveModulePosition[] lastModulePositions = // For delta tracking
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  private SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, new Pose2d());

  // Odometry class for tracking robot pose
  private SwerveDriveOdometry odometry =
      new SwerveDriveOdometry(kinematics, rawGyroRotation, lastModulePositions);

  private SysIdRoutine sysId;
  private SysIdRoutine turnRoutine;

  @AutoLogOutput @Getter private static int reefPoseIndex;

  private double lastTime = Timer.getFPGATimestamp();
  private double deltaTime = 0;
  private Rotation2d lastGyroRotation = new Rotation2d();

  // Things that will be shown on Elastic Dashboard
  private Field2d field;

  private LoggedNetworkNumber matchTime;
  private LoggedNetworkNumber rotation;

  public Drive(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO,
      VisionIO visionIO) {
    this.gyroIO = gyroIO;
    modules[0] = new Module(flModuleIO, 0);
    modules[1] = new Module(frModuleIO, 1);
    modules[2] = new Module(blModuleIO, 2);
    modules[3] = new Module(brModuleIO, 3);
    SparkMaxOdometryThread.getInstance().start();

    this.visionIO = visionIO;

    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      e.printStackTrace();
    }

    // PID Constants used in AutoBuilder config
    PIDConstants translationPID =
        new PIDConstants(
            kPathplannerTranslationP, kPathplannerTranslationI, kPathplannerTranslationD);
    PIDConstants rotationPID =
        new PIDConstants(kPathplannerTurnAngleP, kPathplannerTurnAngleI, kPathplannerTurnAngleD);

    // Configure AutoBuilder for PathPlanner
    AutoBuilder.configure(
        this::getPose, // Robot pose supplier
        this::setPose, // Method to reset odometry (will be called if your auto has a starting pose)
        () ->
            kinematics.toChassisSpeeds(
                getModuleStates()), // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        (speeds, feedforwards) ->
            runVelocity(
                speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds.
        // Also optionally outputs individual module feedforwards
        new PPHolonomicDriveController( // PPHolonomicController is the built in path following
            // controller for holonomic drive trains
            translationPID, // Translation PID constants
            rotationPID // Rotation PID constants
            ),
        config, // The robot configuration
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this // Reference to this subsystem to set requirements
        );

    Pathfinding.setPathfinder(new LocalADStarAK());
    Pathfinding.ensureInitialized();

    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput(
              "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });

    // Configure SysId
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Drive/DriveSysIdTestState", state.toString())),
            new SysIdRoutine.Mechanism(
                volts -> {
                  for (Module module : modules) {
                    module.runCharacterization(volts.in(Volts), 0);
                  }
                },
                null,
                this));

    turnRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Drive/TurnSysIdTestState", state.toString())),
            new SysIdRoutine.Mechanism(
                volts -> {
                  for (Module module : modules) {
                    module.runCharacterization(0, volts.in(Volts));
                  }
                },
                null,
                this));

    // Things that will be shown on elastic

    // Field with robot position
    field = new Field2d();

    SmartDashboard.putData("Field", field);

    // Swerve drive states
    SmartDashboard.putData(
        "Swerve Drive",
        new Sendable() {
          @Override
          public void initSendable(SendableBuilder builder) {
            builder.setSmartDashboardType("SwerveDrive");

            builder.addDoubleProperty(
                "Front Left Angle", () -> modules[0].getAngle().getRadians(), null);
            builder.addDoubleProperty(
                "Front Left Velocity", () -> modules[0].getVelocityMetersPerSec(), null);

            builder.addDoubleProperty(
                "Front Right Angle", () -> modules[1].getAngle().getRadians(), null);
            builder.addDoubleProperty(
                "Front Right Velocity", () -> modules[1].getVelocityMetersPerSec(), null);

            builder.addDoubleProperty(
                "Back Left Angle", () -> modules[2].getAngle().getRadians(), null);
            builder.addDoubleProperty(
                "Back Left Velocity", () -> modules[2].getVelocityMetersPerSec(), null);

            builder.addDoubleProperty(
                "Back Right Angle", () -> modules[3].getAngle().getRadians(), null);
            builder.addDoubleProperty(
                "Back Right Velocity", () -> modules[3].getVelocityMetersPerSec(), null);

            builder.addDoubleProperty(
                "Robot Angle", () -> AllianceFlipUtil.apply(getRotation()).getRadians(), null);
          }
        });

    // Match timer
    matchTime = new LoggedNetworkNumber("/SmartDashboard/Match Time");

    // Robot rotation
    rotation = new LoggedNetworkNumber("SmartDashboard/Robot Angle");
  }

  public void periodic() {
    deltaTime = Timer.getFPGATimestamp() - lastTime;
    lastTime = Timer.getFPGATimestamp();

    odometryLock.lock(); // Prevents odometry updates while reading data
    gyroIO.updateInputs(gyroInputs);
    for (var module : modules) {
      module.updateInputs();
    }
    odometryLock.unlock();
    Logger.processInputs("Drive/Gyro", gyroInputs);

    for (var module : modules) {
      module.periodic();
    }

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
    }
    // Log empty setpoint states when disabled
    if (DriverStation.isDisabled()) {
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
    for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
      modulePositions[moduleIndex] = modules[moduleIndex].getPosition();
    }

    ChassisSpeeds fieldVelocity = getFieldVelocity();
    Logger.recordOutput("FieldVelocity", fieldVelocity);

    // Update gyro angle
    if (gyroInputs.connected) {
      // Use the real gyro angle
      rawGyroRotation = Rotation2d.fromDegrees(gyroIO.getYawAngle());
    } else {
      rawGyroRotation =
          rawGyroRotation.rotateBy(
              Rotation2d.fromRadians(fieldVelocity.omegaRadiansPerSecond * deltaTime));
    }

    Rotation2d dtheta = rawGyroRotation.minus(lastGyroRotation);
    lastGyroRotation = rawGyroRotation;

    odometry.update(rawGyroRotation, modulePositions);

    if (useVision) {
      if (RobotBase.isSimulation()) {
        // Use odometry as "actual robot position" in vision simulation
        visionIO.updateInputs(visionInputs, getPose(), odometry.getPoseMeters());
      } else {
        // Estimate current heading using the previous heading and the change in gyro angle
        visionIO.updateInputs(visionInputs, getPose(), getRotation().rotateBy(dtheta));
      }
      Logger.processInputs("Vision", visionInputs);
      if (visionInputs.hasEstimate) {
        List<Matrix<N3, N1>> stdDeviations = visionIO.getStdArray(visionInputs, getPose());

        for (int i = 0; i < visionInputs.positionEstimates.length; i++) {
          Matrix<N3, N1> allStdDevs = stdDeviations.get(i);
          Matrix<N3, N1> positionStdDevs =
              VecBuilder.fill(allStdDevs.get(0, 0), allStdDevs.get(1, 0), Double.MAX_VALUE);
          Matrix<N3, N1> rotationStdDevs =
              VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, allStdDevs.get(2, 0));

          if (visionInputs.positionEstimates[i].equals(new Pose2d()))
            continue; // Camera i has no estimate
          else if (stdDeviations.size() <= i || visionInputs.timestampArray.length <= i)
            continue; // Avoids index out of bounds exceptions
          else {
            // Position and rotation estimates are done separately by different algorithms
            poseEstimator.addVisionMeasurement(
                visionInputs.positionEstimates[i], visionInputs.timestampArray[i], positionStdDevs);
            poseEstimator.addVisionMeasurement(
                new Pose2d(0, 0, visionInputs.rotationEstimates[i]),
                visionInputs.timestampArray[i],
                rotationStdDevs);
          }
        }
      }
    }

    poseEstimator.updateWithTime(Timer.getFPGATimestamp(), rawGyroRotation, modulePositions);

    Logger.recordOutput("Odometry/Odometry", odometry.getPoseMeters());

    // Update Elastic things
    field.setRobotPose(getPose());

    matchTime.set(Timer.getMatchTime());
    rotation.set(AllianceFlipUtil.apply(getRotation()).getDegrees());
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void runVelocity(ChassisSpeeds speeds) {
    // Calculate module setpoints
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, kMaxSpeedMetersPerSecond);

    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);

    // Send setpoints to modules
    SwerveModuleState[] optimizedSetpointStates = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      // The module returns the optimized state, useful for logging
      optimizedSetpointStates[i] = modules[i].runSetpoint(setpointStates[i]);
    }

    // Log setpoint states
    Logger.recordOutput("SwerveStates/SetpointsOptimized", optimizedSetpointStates);
  }

  /** Stops the drive. */
  public void stop() {
    for (int i = 0; i < 4; i++) {
      modules[i].stop();
    }

    // Log empty setpoints
    Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
    Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
  }

  /** Resets the yaw angle of the estimated position */
  public void resetYaw() {
    setPose(new Pose2d(getPose().getTranslation(), AllianceFlipUtil.apply(new Rotation2d())));
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = getModuleTranslations()[i].getAngle();
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  /** Runs forwards at the commanded voltage. */
  public void runCharacterization(double volts) {
    for (int i = 0; i < 4; i++) {
      modules[i].runCharacterization(volts);
    }
  }

  /** Returns the average drive velocity in radians/sec. */
  public double getCharacterizationVelocity() {
    double driveVelocityAverage = 0.0;
    for (var module : modules) {
      driveVelocityAverage += module.getFFCharacterizationVelocity();
    }
    return driveVelocityAverage / 4.0;
  }

  /** Returns the module states (turn angles and driveZ velocities) for all of the modules. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      positions[i] = modules[i].getPosition();
    }
    return positions;
  }

  /**
   * Gets the current field-relative velocity (x, y and omega) of the robot
   *
   * @return A ChassisSpeeds object of the current field-relative velocity
   */
  public ChassisSpeeds getFieldVelocity() {
    // ChassisSpeeds has a method to convert from field-relative to robot-relative speeds,
    // but not the reverse.  However, because this transform is a simple rotation, negating the
    // angle
    // given as the robot angle reverses the direction of rotation, and the conversion is reversed.
    return ChassisSpeeds.fromRobotRelativeSpeeds(
        kinematics.toChassisSpeeds(getModuleStates()), getRotation());
  }

  /** Returns the position of each module in radians. */
  public double[] getWheelRadiusCharacterizationPositions() {
    double[] values = new double[4];
    for (int i = 0; i < 4; i++) {
      values[i] = modules[i].getWheelRadiusCharacterizationPosition();
    }
    return values;
  }

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public Pose2d getOdometryPose() {
    return odometry.getPoseMeters();
  }

  /** Returns the current odometry rotation. */
  /* public Rotation2d getRotation() {
    return getPose().getRotation();
  } */
  @AutoLogOutput(key = "Drive/Rotation")
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    gyroIO.setYawAngle(pose.getRotation().getDegrees());
    rawGyroRotation = pose.getRotation();
    lastGyroRotation = pose.getRotation();

    // Yes I know it says that you don't need to reset the gyro rotation, but it tweaks out if you
    // don't
    poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
    odometry.resetPosition(rawGyroRotation, getModulePositions(), pose);
  }

  /**
   * Adds a vision measurement to the pose estimator.
   *
   * @param visionPose The pose of the robot as measured by the vision camera.
   * @param timestamp The timestamp of the vision measurement in seconds.
   */
  public void addVisionMeasurement(Pose2d visionPose, double timestamp) {
    poseEstimator.addVisionMeasurement(visionPose, timestamp);
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysId.quasistatic(direction);
  }

  public Command turnQuasistatic(SysIdRoutine.Direction direction) {
    return turnRoutine.quasistatic(direction);
  }

  public Command turnDynamic(SysIdRoutine.Direction direction) {
    return turnRoutine.dynamic(direction);
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysId.dynamic(direction);
  }

  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    return kMaxSpeedMetersPerSecond;
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    return MAX_ANGULAR_SPEED;
  }

  /** Returns an array of module translations. */
  public static Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
      new Translation2d(kTrackWidthX / 2.0, kTrackWidthY / 2.0),
      new Translation2d(kTrackWidthX / 2.0, -kTrackWidthY / 2.0),
      new Translation2d(-kTrackWidthX / 2.0, kTrackWidthY / 2.0),
      new Translation2d(-kTrackWidthX / 2.0, -kTrackWidthY / 2.0)
    };
  }

  /* Configure trajectory following */
  public Command pathfindToPose(Pose2d target_pose, double end_velocity) {
    return AutoBuilder.pathfindToPose(target_pose, kPathConstraints, end_velocity);
  }

  public Command pathfindToPose(Pose2d target_pose) {
    return AutoBuilder.pathfindToPose(target_pose, kPathConstraints, 0.0);
  }

  public Command getAuto(String nameString) {
    return AutoBuilder.buildAuto(nameString);
  }

  public Command runTrajectory(PathPlannerPath path) {
    return AutoBuilder.followPath(path);
  }

  public Command pathfindToTrajectory(PathPlannerPath path) {
    return AutoBuilder.pathfindThenFollowPath(path, kPathConstraints);
  }

  /**
   * This function is flawed because getPose only runs once so the path always starts from the
   * starting pose. Do not use this function until we fix it, use pathfindToPose instead
   */
  public Command splinePathToPose(Pose2d endPose) {
    List<Waypoint> bezierPoints = PathPlannerPath.waypointsFromPoses(getPose(), endPose);

    // Create the path using the bezier points created above
    PathPlannerPath path =
        new PathPlannerPath(
            bezierPoints,
            kPathConstraints, // The constraints for this path. If using a differential drivetrain,
            // the angular constraints have no effect.
            new IdealStartingState(0.0, getPose().getRotation()),
            new GoalEndState(
                0.0,
                Rotation2d.fromDegrees(
                    -90)) // Goal end state. You can set a holonomic rotation here. If using a
            // differential drivetrain, the rotation will have no effect.
            );
    return AutoBuilder.followPath(path);
  }

  public Command driveToFieldPosition(String positionName) {
    try {
      return AutoBuilder.pathfindThenFollowPath(
          PathPlannerPath.fromPathFile(positionName + "-align"), kPathConstraints);
    } catch (Exception e) {
      e.printStackTrace();
      return new InstantCommand();
    }
  }

  /**
   * Follows PathPlanner GUI-created path from file
   *
   * @param filename the name of the file read
   * @return a command that follows the path
   */
  public Command followPathFileCommand(String filename) {
    try {
      return AutoBuilder.followPath(PathPlannerPath.fromPathFile(filename));
    } catch (Exception e) {
      e.printStackTrace();
      return new InstantCommand();
    }
  }

  public void increaseReefPoseIndex() {
    if (reefPoseIndex < 11) {
      reefPoseIndex += 1;
    } else {
      reefPoseIndex = 0;
    }
  }

  public void decreaseReefPoseIndex() {
    if (reefPoseIndex > 0) {
      reefPoseIndex -= 1;
    } else {
      reefPoseIndex = 11;
    }
  }

  public Command reefPoseChooserIncrement() {
    return new InstantCommand(() -> increaseReefPoseIndex());
  }

  public Command reefPoseChooserDecrement() {
    return new InstantCommand(() -> decreaseReefPoseIndex());
  }

  public Command driveToReef() {
    return new ProxyCommand(
            () -> pathfindToPose(FieldConstants.ReefScoringPositions[reefPoseIndex]))
        .withName("Pathfind " + reefPoseIndex);
  }

  /**
   * A command that automatically aligns to the closest reef position
   *
   * @return
   */
  public Command alignToReef() {
    return new AlignToPose(
        this,
        () -> {
          Pose2d[] reefPoses = FieldConstants.ReefScoringPositions;
          Pose2d currentPose = getPose();

          int closestPose = 0;
          double closestDistance = Double.MAX_VALUE;

          // Calculate distance to each reef pose and select closest one
          for (int i = 0; i < 12; i++) {
            Transform2d currentToTarget = AllianceFlipUtil.apply(reefPoses[i]).minus(currentPose);
            double distance = currentToTarget.getTranslation().getNorm();

            if (distance < closestDistance) {
              closestPose = i;
              closestDistance = distance;
            }
          }

          return AllianceFlipUtil.apply(reefPoses[closestPose]);
        },
        false);
  }

  /**
   * A command that automatically aligns to the closest reef center position
   *
   * @return
   */
  public Command alignToCenterReef() {
    return new AlignToPose(
        this,
        () -> {
          Pose2d[] reefPoses = FieldConstants.ReefCenterPositions;
          Pose2d currentPose = getPose();

          int closestPose = 0;
          double closestDistance = Double.MAX_VALUE;

          // Calculate distance to each reef pose and select closest one
          for (int i = 0; i < 6; i++) {
            Transform2d currentToTarget = AllianceFlipUtil.apply(reefPoses[i]).minus(currentPose);
            double distance = currentToTarget.getTranslation().getNorm();

            if (distance < closestDistance) {
              closestPose = i;
              closestDistance = distance;
            }
          }

          return AllianceFlipUtil.apply(FieldConstants.ReefCenterPositions[closestPose]);
        },
        false);
  }

  /**
   * A command that automatically aligns to the closest reef left scoring location
   *
   * @return
   */
  public Command alignToReefLeft() {
    return new AlignToPose(
        this,
        () -> {
          Pose2d[] reefCenterPoses = FieldConstants.Reef.centerFaces;
          Pose2d currentPose = getPose();

          int closestPose = 0;
          double closestDistance = Double.MAX_VALUE;

          // Calculate distance to each reef pose and select closest one
          for (int i = 0; i < 6; i++) {
            Transform2d currentToTarget =
                AllianceFlipUtil.apply(reefCenterPoses[i]).minus(currentPose);
            double distance = currentToTarget.getTranslation().getNorm();

            if (distance < closestDistance) {
              closestPose = i;
              closestDistance = distance;
            }
          }

          return AllianceFlipUtil.apply(FieldConstants.ReefScoringPositions[closestPose * 2 + 1]);
        },
        false);
  }

  /**
   * A command that automatically aligns to the closest reef right scoring location
   *
   * @return
   */
  public Command alignToReefRight() {
    return new AlignToPose(
        this,
        () -> {
          Pose2d[] reefCenterPoses = FieldConstants.Reef.centerFaces;
          Pose2d currentPose = getPose();

          int closestPose = 0;
          double closestDistance = Double.MAX_VALUE;

          // Calculate distance to each reef pose and select closest one
          for (int i = 0; i < 6; i++) {
            Transform2d currentToTarget =
                AllianceFlipUtil.apply(reefCenterPoses[i]).minus(currentPose);
            double distance = currentToTarget.getTranslation().getNorm();

            if (distance < closestDistance) {
              closestPose = i;
              closestDistance = distance;
            }
          }

          return AllianceFlipUtil.apply(FieldConstants.ReefScoringPositions[closestPose * 2]);
        },
        false);
  }

  /**
   * A command that automatically aligns to a specific reef position
   *
   * @param reefPosition The desired position to align to
   * @param reefLevel The desired level to score on
   * @return the command, idk what else to put here
   */
  public Command alignToReefAuto(ReefPositions reefPosition, ReefLevels reefLevel) {
    return new AlignToPose(
        this,
        () -> {
          int index = Integer.parseInt(reefPosition.toString().substring(1)) - 1;
          Pose2d pose = AllianceFlipUtil.apply(FieldConstants.ReefScoringPositions[index]);

          // If you're scoring on L1, move a bit forward and go to the center of the reef face
          if (reefLevel == ReefLevels.l1) {
            pose = FieldConstants.translateCoordinates(pose, pose.getRotation().getDegrees(), 0.1);

            // Move pose to center of reef face
            // If you're on the right, move left, else move right
            if (index % 2 == 0) {
              pose =
                  FieldConstants.translateCoordinates(
                      pose,
                      pose.getRotation().getDegrees() + 90,
                      FieldConstants.reefFaceCenterToScoreDistance);
            } else {
              pose =
                  FieldConstants.translateCoordinates(
                      pose,
                      pose.getRotation().getDegrees() - 90,
                      FieldConstants.reefFaceCenterToScoreDistance);
            }
          }

          return pose;
        },
        true);
  }

  /**
   * A command that automatically aligns to the closest coral station
   *
   * @return
   */
  public Command alignToStation() {
    return new AlignToPose(
        this,
        () -> {
          if (AllianceFlipUtil.apply(getPose()).getY() > FieldConstants.fieldWidth / 2) {
            return AllianceFlipUtil.apply(FieldConstants.STATION_POSITION[0]);
          }
          return AllianceFlipUtil.apply(FieldConstants.STATION_POSITION[1]);
        },
        false);
  }

  /**
   * A command that automatically aligns to the closest coral station
   *
   * @return
   */
  public Command alignToStationAuto() {
    return new AlignToPose(
        this,
        () -> {
          if (AllianceFlipUtil.apply(getPose()).getY() > FieldConstants.fieldWidth / 2) {
            return AllianceFlipUtil.apply(FieldConstants.STATION_POSITION[0]);
          }
          return AllianceFlipUtil.apply(FieldConstants.STATION_POSITION[1]);
        },
        true);
  }
}
