package frc.robot.util.drive;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.util.drive.CommandSnailController.DPad;
import java.util.function.DoubleSupplier;

public class DriveControls {
  // Controllers
  public static final CommandSnailController driver = new CommandSnailController(0);
  public static final CommandSnailController operator = new CommandSnailController(1);

  // Useful for things that don't need to be triggered
  private static final Trigger EMPTY_TRIGGER = new Trigger(() -> false);
  private static final DoubleSupplier EMPTY_DOUBLE_SUPPLIER = () -> 0.0;

  // Drive controls
  public static DoubleSupplier DRIVE_FORWARD;
  public static DoubleSupplier DRIVE_STRAFE;
  public static DoubleSupplier DRIVE_ROTATE;
  public static Trigger DRIVE_SLOW;
  public static Trigger DRIVE_STOP;
  public static Trigger DRIVE_ROBOT_RELATIVE;

  // Algae Pivot Controls
  public static DoubleSupplier ALGAE_PIVOT_SPEED;
  public static Trigger ALGAE_PIVOT_STOW;
  public static Trigger ALGAE_PIVOT_DOWN;
  public static Trigger ALGAE_PIVOT_PROCESSOR;

  // Coral pivot controls
  public static DoubleSupplier CORAL_PIVOT_ROTATE;
  public static Trigger CORAL_PIVOT_L1;
  public static Trigger CORAL_PIVOT_L2_L3;
  public static Trigger CORAL_PIVOT_STOW;
  public static Trigger CORAL_PIVOT_INTAKE;

  // Drive Turns
  public static Trigger TURN_90;
  public static Trigger TURN_180;

  // Rumble Controls
  public static Trigger TIMED_RUMBLE;
  public static Trigger INTAKE_RUMBLE;

  // Potential Hail Marry Program [Suggested by Owen]
  public static Trigger SHOOT_FROM_SOURCE;

  // Algae Intake Controls
  public static Trigger INTAKE_ALGAE;
  public static Trigger SHOOT_ALGAE;

  // Coral Intake Controls
  public static Trigger INTAKE_CORAL;
  public static Trigger SHOOT_CORAL;

  // Elevator Controls
  public static DoubleSupplier ELEVATOR_SPEED;
  public static Trigger ELEVATOR_L1;
  public static Trigger ELEVATOR_L2;
  public static Trigger ELEVATOR_L3;
  public static Trigger ELEVATOR_DOWN;
  public static Trigger ELEVATOR_INTAKE;

  // Setup the controls
  public static void configureControls() {
    switch (Constants.driver) {
      case PROGRAMMERS:
      default:
        DRIVE_FORWARD = () -> -driver.getLeftY();
        DRIVE_STRAFE = () -> -driver.getLeftX();
        DRIVE_ROTATE = () -> -driver.getRightX();
        DRIVE_STOP = driver.x();
        DRIVE_SLOW = driver.rightBumper();
        DRIVE_ROBOT_RELATIVE = EMPTY_TRIGGER;

        TURN_90 = driver.y();
        TURN_180 = driver.start();
        break;
    }

    switch (Constants.operator) {
      case PROGRAMMERS:
      default:
        // Operator controls
        ALGAE_PIVOT_SPEED = () -> operator.getLeftYD();
        ALGAE_PIVOT_DOWN = operator.a();
        ALGAE_PIVOT_STOW = operator.b();
        ALGAE_PIVOT_PROCESSOR = operator.x();

        INTAKE_CORAL = operator.a();
        SHOOT_CORAL = operator.b();

        CORAL_PIVOT_ROTATE = () -> (operator.getLeftY());
        CORAL_PIVOT_L1 = operator.x();
        CORAL_PIVOT_L2_L3 = operator.a();
        CORAL_PIVOT_STOW = operator.b();
        CORAL_PIVOT_INTAKE = operator.y();

        INTAKE_ALGAE = operator.leftBumper();
        SHOOT_ALGAE = operator.rightBumper();

        ELEVATOR_SPEED = () -> operator.getLeftYD();
        ELEVATOR_L1 = operator.getDPad(DPad.LEFT);
        ELEVATOR_L2 = operator.getDPad(DPad.RIGHT);
        ELEVATOR_L3 = operator.getDPad(DPad.UP);
        ELEVATOR_DOWN = operator.b();
        ELEVATOR_INTAKE = operator.getDPad(DPad.DOWN);
        break;

        // bottom right Left joystick to intake
    }
  }

  private static Command getRumbleCommand(CommandSnailController driver) {
    return new InstantCommand(() -> driver.rumble(1))
        .andThen(new WaitCommand(1))
        .andThen(() -> driver.rumble(0));
  }

  public static Command getRumbleBoth() {
    return getRumbleCommand(driver).alongWith(getRumbleCommand(operator));
  }
}
