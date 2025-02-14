// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.util.drive.DriveControls.*;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.FeedForwardCharacterization;
import frc.robot.subsystems.algaePivot.AlgaePivot;
import frc.robot.subsystems.algaePivot.AlgaePivotConstants;
import frc.robot.subsystems.algaePivot.AlgaePivotIO;
import frc.robot.subsystems.algaePivot.AlgaePivotIOSim;
import frc.robot.subsystems.algaePivot.AlgaePivotIOSparkMax;
import frc.robot.subsystems.algaeIntake.AlgaeIntake;
import frc.robot.subsystems.algaeIntake.AlgaeIntakeConstants;
import frc.robot.subsystems.algaeIntake.AlgaeIntakeIO;
import frc.robot.subsystems.algaeIntake.AlgaeIntakeIOSim;
import frc.robot.subsystems.algaeIntake.AlgaeIntakeIOSparkMax;
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
            drive, drive::runCharacterizationVolts, drive::getCharacterizationVelocity));

    // Configure the button bindings
    configureControls();
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(drive, DRIVE_FORWARD, DRIVE_STRAFE, DRIVE_ROTATE));
    algaePivot.setDefaultCommand(algaePivot.ManualCommand(ALGAE_PIVOT_SPEED));

    DRIVE_SLOW.onTrue(new InstantCommand(DriveCommands::toggleSlowMode));

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
    INTAKE_ALGAE.whileTrue(algaeIntake.manualCommand(AlgaeIntakeConstants.ALGAE_INTAKE_IN_VOLTAGE));
    SHOOT_ALGAE.whileTrue(algaeIntake.manualCommand(AlgaeIntakeConstants.ALGAE_INTAKE_OUT_VOLTAGE));

    // Coral Intake Controls
    INTAKE_CORAL.whileTrue(coralIntake.ManualCommand(CoralIntakeConstants.CORAL_INTAKE_IN_VOLTAGE));
    SHOOT_CORAL.whileTrue(coralIntake.ManualCommand(CoralIntakeConstants.CORAL_INTAKE_OUT_VOLTAGE));

    coralPivot.setDefaultCommand(coralPivot.ManualCommand(CORAL_PIVOT_ROTATE));
    CORAL_PIVOT_L1.onTrue(coralPivot.InstantPIDCommand(CoralPivotConstants.CORAL_PIVOT_L1_ANGLE));
    CORAL_PIVOT_L2_L3.onTrue(
        coralPivot.InstantPIDCommand(CoralPivotConstants.CORAL_PIVOT_L2_L3_ANGLE));
    CORAL_PIVOT_INTAKE.onTrue(
        coralPivot.InstantPIDCommand(CoralPivotConstants.CORAL_PIVOT_INTAKE_ANGLE));
    CORAL_PIVOT_STOW.onTrue(
        coralPivot.InstantPIDCommand(CoralPivotConstants.CORAL_PIVOT_STOW_ANGLE));

    elevator.setDefaultCommand(elevator.ManualCommand(ELEVATOR_SPEED));
    ELEVATOR_L1.onTrue(elevator.InstantPIDCommand(ElevatorConstants.ELEVATOR_L1_HEIGHT));
    ELEVATOR_L2.onTrue(elevator.InstantPIDCommand(ElevatorConstants.ELEVATOR_L2_HEIGHT));
    ELEVATOR_L3.onTrue(elevator.InstantPIDCommand(ElevatorConstants.ELEVATOR_L3_HEIGHT));
    ELEVATOR_INTAKE.onTrue(elevator.InstantPIDCommand(ElevatorConstants.ELEVATOR_INTAKE_HEIGHT));
    ELEVATOR_DOWN.onTrue(elevator.InstantPIDCommand(0));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
