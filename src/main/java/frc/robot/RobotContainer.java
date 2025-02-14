// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.util.drive.DriveControls.*;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.FeedForwardCharacterization;
// this is for the coralPivot
import frc.robot.subsystems.coralPivot.CoralPivot;
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
  private final CoralPivot coralPivot;
  private final Elevator elevator;

  private Mechanism2d coralPivotMech = new Mechanism2d(3, 3);
  private Mechanism2d elevatorMech = new Mechanism2d(3, 3);

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

        coralPivot = new CoralPivot(new CoralPivotIO() {});
        elevator = new Elevator(new ElevatorIO() {});
        break;
    }

    // Set up robot state manager

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

    DRIVE_SLOW.onTrue(new InstantCommand(DriveCommands::toggleSlowMode));

    DRIVE_STOP.onTrue(
        new InstantCommand(
            () -> {
              drive.stopWithX();
              drive.resetYaw();
            },
            drive));

    coralPivot.setDefaultCommand(coralPivot.ManualCommand(CORAL_PIVOT_ROTATE));
    CORAL_PIVOT_L2_3.onTrue(coralPivot.InstantPIDCommand(-0.2));
    CORAL_PIVOT_DOWN.onTrue(coralPivot.InstantPIDCommand(Units.degreesToRadians(-70)));

    elevator.setDefaultCommand(elevator.ManualCommand(ELEVATOR_SPEED));
    ELEVATOR_L1.onTrue(elevator.InstantPIDCommand(0.5));
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
