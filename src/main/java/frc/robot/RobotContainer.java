// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.util.drive.DriveControls.*;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.FeedForwardCharacterization;
import frc.robot.subsystems.algaeIntake.AlgaeIntake;
import frc.robot.subsystems.algaeIntake.AlgaeIntakeConstants;
import frc.robot.subsystems.algaeIntake.AlgaeIntakeIO;
import frc.robot.subsystems.algaeIntake.AlgaeIntakeIOSim;
import frc.robot.subsystems.algaeIntake.AlgaeIntakeIOSparkMax;
import frc.robot.subsystems.algaePivot.AlgaePivot;
import frc.robot.subsystems.algaePivot.AlgaePivotConstants;
import frc.robot.subsystems.algaePivot.AlgaePivotIO;
import frc.robot.subsystems.algaePivot.AlgaePivotIOSim;
import frc.robot.subsystems.algaePivot.AlgaePivotIOSparkMax;
import frc.robot.subsystems.coralIntake.CoralIntake;
import frc.robot.subsystems.coralIntake.CoralIntakeConstants;
import frc.robot.subsystems.coralIntake.CoralIntakeIO;
import frc.robot.subsystems.coralIntake.CoralIntakeIOSim;
import frc.robot.subsystems.coralIntake.CoralIntakeIOSparkMax;
import frc.robot.subsystems.coralPivot.CoralPivot;
import frc.robot.subsystems.coralPivot.CoralPivotConstants;
import frc.robot.subsystems.coralPivot.CoralPivotIO;
import frc.robot.subsystems.coralPivot.CoralPivotIOSim;
import frc.robot.subsystems.coralPivot.CoralPivotIOSparkMax;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOReal;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSparkMax;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.ElevatorIOSparkMax;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhoton;
import frc.robot.subsystems.vision.VisionIOSim;
import frc.robot.util.autonomous.CustomAutoChooser;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final AlgaePivot algaePivot;
  private final AlgaeIntake algaeIntake;
  private final CoralIntake coralIntake;
  private final CoralPivot coralPivot;
  private final Elevator elevator;

  private Mechanism2d coralPivotMech = new Mechanism2d(3, 3);
  private Mechanism2d elevatorMech = new Mechanism2d(3, 3);
  private Mechanism2d algaePivotMech = new Mechanism2d(3, 3);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;
  private final CustomAutoChooser customAutoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
        // Real robot, instantiate hardware IO implementations
      case REAL:
        drive =
            new Drive(
                new GyroIOReal(),
                new ModuleIOSparkMax(0),
                new ModuleIOSparkMax(1),
                new ModuleIOSparkMax(2),
                new ModuleIOSparkMax(3),
                new VisionIOPhoton());
        algaePivot = new AlgaePivot(new AlgaePivotIOSparkMax());
        algaeIntake = new AlgaeIntake(new AlgaeIntakeIOSparkMax());
        coralIntake = new CoralIntake(new CoralIntakeIOSparkMax());
        coralPivot = new CoralPivot(new CoralPivotIOSparkMax());
        elevator = new Elevator(new ElevatorIOSparkMax());
        break;

        // Sim robot, instantiate physics sim IO implementations
      case SIM:
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new VisionIOSim());
        algaePivot = new AlgaePivot(new AlgaePivotIOSim());
        algaeIntake = new AlgaeIntake(new AlgaeIntakeIOSim());
        coralIntake = new CoralIntake(new CoralIntakeIOSim());
        coralPivot = new CoralPivot(new CoralPivotIOSim());
        elevator = new Elevator(new ElevatorIOSim());
        break;

        // Replayed robot, disable IO implementations
      default:
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new VisionIO() {});
        algaePivot = new AlgaePivot(new AlgaePivotIO() {});
        algaeIntake = new AlgaeIntake(new AlgaeIntakeIO() {});
        coralIntake = new CoralIntake(new CoralIntakeIO() {});
        coralPivot = new CoralPivot(new CoralPivotIO() {});
        elevator = new Elevator(new ElevatorIO() {});
        break;
    }

    // Set up robot state manager

    MechanismRoot2d algaePivotRoot = algaePivotMech.getRoot("pivot", 1, 0.5);
    algaePivotRoot.append(algaePivot.getArmMechanism());
    // add subsystem mechanisms
    SmartDashboard.putData("Algae Pivot Mechanism", algaePivotMech);

    MechanismRoot2d coralPivotRoot = coralPivotMech.getRoot("coral pivot", 1, 0.5);
    coralPivotRoot.append(coralPivot.getArmMechanism());
    // add subsystem mechanisms
    SmartDashboard.putData("Coral Pivot Mechanism", coralPivotMech);

    MechanismRoot2d elevatorRoot = elevatorMech.getRoot("elevator", 1, 0.5);
    elevatorRoot.append(elevator.getElevatorMechanism());
    // add subsystem mechanisms
    SmartDashboard.putData("Elevator Mechanism", elevatorMech);

    // Set up auto routines
    /* NamedCommands.registerCommand(
    "Run Flywheel",
    Commands.startEnd(
            () -> flywheel.runVelocity(flywheelSpeedInput.get()), flywheel::stop, flywheel)
        .withTimeout(5.0)); */
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up feedforward characterization
    autoChooser.addOption(
        "Drive FF Characterization",
        new FeedForwardCharacterization(
            drive, drive::runCharacterization, drive::getCharacterizationVelocity));

    customAutoChooser = new CustomAutoChooser(this, drive, algaePivot);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  public void configureButtonBindings() {
    configureControls();

    drive.setDefaultCommand(
        DriveCommands.joystickDrive(drive, DRIVE_FORWARD, DRIVE_STRAFE, DRIVE_ROTATE));
    algaePivot.setDefaultCommand(algaePivot.ManualCommand(ALGAE_PIVOT_SPEED));

    // DRIVE_SLOW.onTrue(new InstantCommand(DriveCommands::toggleSlowMode));

    DRIVE_STOP.onTrue(
        new InstantCommand(
            () -> {
              drive.stopWithX();
              drive.resetYaw();
            },
            drive));

    ALGAE_PIVOT_DOWN.onTrue(
        algaePivot.InstantPIDCommand(AlgaePivotConstants.ALGAE_PIVOT_DOWN_ANGLE));
    ALGAE_PIVOT_STOW.onTrue(
        algaePivot.InstantPIDCommand(AlgaePivotConstants.ALGAE_PIVOT_STOW_ANGLE));
    ALGAE_PIVOT_PROCESSOR.onTrue(
        algaePivot.InstantPIDCommand(AlgaePivotConstants.ALGAE_PIVOT_PROCESSOR_ANGLE));

    // Algae Intake Controls
    INTAKE_ALGAE.whileTrue(algaeIntake.manualCommand(AlgaeIntakeConstants.ALGAE_INTAKE_IN_SPEED));
    EJECT_ALGAE.whileTrue(algaeIntake.manualCommand(AlgaeIntakeConstants.ALGAE_INTAKE_OUT_SPEED));

    // Coral Intake Controls
    INTAKE_CORAL.whileTrue(coralIntake.ManualCommand(CoralIntakeConstants.CORAL_INTAKE_IN_SPEED));
    EJECT_CORAL.whileTrue(coralIntake.ManualCommand(CoralIntakeConstants.CORAL_INTAKE_OUT_SPEED));

    coralPivot.setDefaultCommand(coralPivot.ManualCommand(CORAL_PIVOT_SPEED));
    CORAL_PIVOT_L1.onTrue(coralPivot.InstantPIDCommand(CoralPivotConstants.CORAL_PIVOT_L1_ANGLE));
    CORAL_PIVOT_L2_L3.onTrue(
        coralPivot.InstantPIDCommand(CoralPivotConstants.CORAL_PIVOT_L2_L3_ANGLE));
    CORAL_PIVOT_STATION.onTrue(
        coralPivot.InstantPIDCommand(CoralPivotConstants.CORAL_PIVOT_STATION_ANGLE));
    CORAL_PIVOT_STOW.onTrue(
        coralPivot.InstantPIDCommand(CoralPivotConstants.CORAL_PIVOT_STOW_ANGLE));

    elevator.setDefaultCommand(elevator.HoldSetpointCommand());
    ELEVATOR_L1.onTrue(elevator.InstantPIDCommand(ElevatorConstants.ELEVATOR_L1_HEIGHT));
    ELEVATOR_L2.onTrue(elevator.InstantPIDCommand(ElevatorConstants.ELEVATOR_L2_HEIGHT));
    ELEVATOR_L3.onTrue(elevator.InstantPIDCommand(ElevatorConstants.ELEVATOR_L3_HEIGHT));
    ELEVATOR_STATION.onTrue(elevator.InstantPIDCommand(ElevatorConstants.ELEVATOR_STATION_HEIGHT));
    ELEVATOR_DOWN.onTrue(elevator.InstantPIDCommand(ElevatorConstants.ELEVATOR_MIN_HEIGHT));

    COMBINED_L1.onTrue(goToL1());
    COMBINED_L2.onTrue(goToL2());
    COMBINED_L3.onTrue(goToL3());
    COMBINED_STATION.onTrue(goToStation());
    // COMBINED_STOW.onTrue(stow());

    TOGGLE_REEF_POSITION_UP.onTrue(drive.reefPoseChooserIncrement());
    TOGGLE_REEF_POSITION_DOWN.onTrue(drive.reefPoseChooserDecrement());
    DRIVE_TO_REEF.onTrue(drive.driveToReef());

    JOYSTICK_REEF_POINT.whileTrue(
        DriveCommands.joystickReefPoint(drive, DRIVE_FORWARD, DRIVE_STRAFE));
    JOYSTICK_STATION_POINT.whileTrue(
        DriveCommands.joystickStationPoint(drive, DRIVE_FORWARD, DRIVE_STRAFE));
    JOYSTICK_PROCESSOR_POINT.whileTrue(
        DriveCommands.joystickProcessorPoint(drive, DRIVE_FORWARD, DRIVE_STRAFE));

    ALIGN_REEF.whileTrue(drive.alignToReef());
    ALIGN_REEF_LEFT.whileTrue(drive.alignToReefLeft());
    ALIGN_REEF_RIGHT.whileTrue(drive.alignToReefRight());
    ALIGN_STATION.whileTrue(drive.alignToStation());
    ALIGN_REEF_CENTER.whileTrue(drive.alignToCenterReef());

    new Trigger(() -> (int) Timer.getMatchTime() == 20.0).onTrue(getRumbleBoth());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // return autoChooser.get();
    return customAutoChooser.getAutoCommand();
    // return DriveCommands.feedforwardCharacterization(drive);
    // return DriveCommands.wheelRadiusCharacterization(drive);
  }

  // Subsystem compound commands
  public Command coralShimmy() {
    return coralIntake
        .ManualCommand(CoralIntakeConstants.CORAL_INTAKE_OUT_SPEED)
        .withTimeout(0.1)
        .andThen(
            coralIntake.ManualCommand(CoralIntakeConstants.CORAL_INTAKE_IN_SPEED).withTimeout(0.4));
  }

  public Command goToL1() {
    return elevator.InstantPIDCommand(ElevatorConstants.ELEVATOR_L1_HEIGHT)
        .alongWith(coralPivot.InstantPIDCommand(CoralPivotConstants.CORAL_PIVOT_L1_ANGLE));
  }

  public Command goToL2() {
    return elevator.InstantPIDCommand(ElevatorConstants.ELEVATOR_L2_HEIGHT)
        .alongWith(coralPivot.InstantPIDCommand(CoralPivotConstants.CORAL_PIVOT_L2_L3_ANGLE));
  }

  public Command goToL3() {
    return elevator.InstantPIDCommand(ElevatorConstants.ELEVATOR_L3_HEIGHT)
        .alongWith(coralPivot.InstantPIDCommand(CoralPivotConstants.CORAL_PIVOT_L2_L3_ANGLE));
  }

  public Command goToStation() {
    return coralPivot
        .InstantPIDCommand(CoralPivotConstants.CORAL_PIVOT_STATION_ANGLE)
        .andThen(elevator.InstantPIDCommand(ElevatorConstants.ELEVATOR_STATION_HEIGHT));
  }

  public Command stow() {
    return elevator.InstantPIDCommand(ElevatorConstants.ELEVATOR_MIN_HEIGHT)
        .alongWith(coralPivot.InstantPIDCommand(CoralPivotConstants.CORAL_PIVOT_STOW_ANGLE));
  }

  public Command goToL1Auto() {
    return coralPivot
        .InstantPIDCommand(CoralPivotConstants.CORAL_PIVOT_L1_ANGLE)
        .andThen(new WaitCommand(0.5))
        .andThen(elevator.InstantPIDCommand(ElevatorConstants.ELEVATOR_L1_HEIGHT));
  }

  public Command goToL2Auto() {
    return coralPivot
        .InstantPIDCommand(CoralPivotConstants.CORAL_PIVOT_L2_L3_ANGLE)
        .andThen(new WaitCommand(0.5))
        .andThen(elevator.InstantPIDCommand(ElevatorConstants.ELEVATOR_L2_HEIGHT));
  }

  public Command goToL3Auto() {
    return coralPivot
        .InstantPIDCommand(CoralPivotConstants.CORAL_PIVOT_L2_L3_ANGLE)
        .andThen(new WaitCommand(0.5))
        .andThen(elevator.InstantPIDCommand(ElevatorConstants.ELEVATOR_L3_HEIGHT));
  }

  public Command stowAuto() {
    return elevator.PIDCommand(ElevatorConstants.ELEVATOR_MIN_HEIGHT)
        .alongWith(coralPivot.PIDCommand(CoralPivotConstants.CORAL_PIVOT_STOW_ANGLE));
  }

  public Command goToStationAuto() {
    return elevator.InstantPIDCommand(ElevatorConstants.ELEVATOR_STATION_HEIGHT)
        .alongWith(coralPivot.InstantPIDCommand(CoralPivotConstants.CORAL_PIVOT_STATION_ANGLE));
  }

  public Command algaeIntake() {
    return algaeIntake
        .manualCommand(AlgaeIntakeConstants.ALGAE_INTAKE_IN_SPEED)
        .until(() -> algaePivot.isBreakBeamBroken())
        .andThen(
            algaeIntake.manualCommand(AlgaeIntakeConstants.ALGAE_INTAKE_IN_SPEED).withTimeout(0.5));
  }

  public Command coralIntake() {
    return coralIntake
        .ManualCommand(CoralIntakeConstants.CORAL_INTAKE_IN_SPEED)
        .withTimeout(0.2)
        .andThen(
            coralIntake
                .ManualCommand(CoralIntakeConstants.CORAL_INTAKE_IN_SPEED)
                .withTimeout(1)
                .until(() -> coralIntake.hasCoral())
                .until(() -> coralIntake.getVelocity() > -0.2));
  }

  public Command coralIntakeForever() {
    return coralIntake.ManualCommand(CoralIntakeConstants.CORAL_INTAKE_IN_SPEED);
  }

  public Command coralOuttakeForever() {
    return coralIntake.ManualCommand(CoralIntakeConstants.CORAL_INTAKE_OUT_SPEED);
  }

  public Command coralFeeder() {
    return coralPivot
        .PIDCommand(CoralPivotConstants.CORAL_PIVOT_STATION_ANGLE)
        .andThen(coralIntake());
  }

  public Command algaeOuttake() {
    return algaeIntake
        .manualCommand(AlgaeIntakeConstants.ALGAE_INTAKE_OUT_SPEED)
        .until(() -> !algaePivot.isBreakBeamBroken());
  }

  public Command coralOuttake() {
    return coralIntake.ManualCommand(CoralIntakeConstants.CORAL_INTAKE_OUT_SPEED).withTimeout(0.25);
  }

  // Moves right for 0.1 seconds, diagonally for 0.4 seconds, and stops for 0.3 seconds
  // All while running the outtake to knock down algae
  public Command knockAlgaeDownFromLeft() {
    // Calculate the x and y distance we want to move so our speed can be in the same direction
    double currentXDistFromReef = FieldConstants.distanceBackFromReef;
    double desiredXDistFromReef = DriveConstants.kRobotWidth / 2;

    double robotSpeed = 0.5; // Half of max speed (like a joystick value of 0.5)

    // We realized that 0.5 m/s isn't actually the speed we're moving at
    // But we're keeping this convoluted calculation method because it works (DON'T TOUCH IT)
    double xDistance = currentXDistFromReef - desiredXDistFromReef;
    double yDistance = -FieldConstants.reefFaceCenterToScoreDistance + robotSpeed * 0.1;

    // Normalize distances and scale them to get diagonal speeds in the same direction
    double magnitude = Math.hypot(xDistance, yDistance);
    double xSpeed = xDistance / magnitude * robotSpeed;
    double ySpeed = yDistance / magnitude * robotSpeed;

    // Drive robot relative using calculated speeds
    Command driveCommand =
        DriveCommands.joystickDriveRobotRelative(drive, () -> 0, () -> -robotSpeed, () -> 0)
            .withTimeout(0.1)
            .andThen(
                DriveCommands.joystickDriveRobotRelative(drive, () -> xSpeed, () -> ySpeed, () -> 0)
                    .withTimeout(0.4));

    return driveCommand
        .deadlineFor(coralOuttakeForever())
        .andThen(
            coralOuttakeForever()
                .withTimeout(0.3)
                .deadlineFor(DriveCommands.joystickDrive(drive, () -> 0, () -> 0, () -> 0)));
  }

  // Moves left for 0.1 seconds, diagonally for 0.4 seconds, and stops for 0.3 seconds
  // All while running the outtake to knock down algae
  public Command knockAlgaeDownFromRight() {
    // Calculate the x and y distance we want to move so our speed can be in the same direction
    double currentXDistFromReef = FieldConstants.distanceBackFromReef;
    double desiredXDistFromReef = DriveConstants.kRobotWidth / 2;

    double robotSpeed = 0.5; // Half of max speed (like a joystick value of 0.5)

    // We realized that 0.5 m/s isn't actually the speed we're moving at
    // But we're keeping this convoluted calculation method because it works (DON'T TOUCH IT)
    double xDistance = currentXDistFromReef - desiredXDistFromReef;
    double yDistance = FieldConstants.reefFaceCenterToScoreDistance - robotSpeed * 0.1;

    // Normalize distances and scale them to get diagonal speeds in the same direction
    double magnitude = Math.hypot(xDistance, yDistance);
    double xSpeed = xDistance / magnitude * robotSpeed;
    double ySpeed = yDistance / magnitude * robotSpeed;

    // Drive robot relative using calculated speeds
    Command driveCommand =
        DriveCommands.joystickDriveRobotRelative(drive, () -> 0, () -> robotSpeed, () -> 0)
            .withTimeout(0.1)
            .andThen(
                DriveCommands.joystickDriveRobotRelative(drive, () -> xSpeed, () -> ySpeed, () -> 0)
                    .withTimeout(0.4));

    return driveCommand
        .deadlineFor(coralOuttakeForever())
        .andThen(
            coralOuttakeForever()
                .withTimeout(0.3)
                .deadlineFor(DriveCommands.joystickDrive(drive, () -> 0, () -> 0, () -> 0)));
  }

  public Command processor() {
    return algaePivot
        .PIDCommand(AlgaePivotConstants.ALGAE_PIVOT_PROCESSOR_ANGLE)
        .andThen(algaeOuttake());
  }

  public Command fullL1() {
    return (goToL1().andThen(coralOuttake()));
  }

  public Command fullL2() {
    return (goToL2().andThen(coralOuttake()));
  }

  public Command fullL3() {
    return (goToL3().andThen(coralOuttake()));
  }

  public Command fullL1Auto() {
    return (goToL1Auto().andThen(coralOuttake()));
  }

  public Command fullL2Auto() {
    return (goToL2Auto().andThen(coralOuttake()));
  }

  public Command fullL3Auto() {
    return (goToL3Auto().andThen(coralOuttake()));
  }
}
