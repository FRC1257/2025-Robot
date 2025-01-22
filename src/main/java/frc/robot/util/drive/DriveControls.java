package frc.robot.util.drive;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
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

  // Drive Turns
  public static Trigger TURN_90;
  public static Trigger TURN_180;

  // Rumble Controls
  public static Trigger TIMED_RUMBLE;
  public static Trigger INTAKE_RUMBLE;

  // Potential Hail Marry Program [Suggested by Owen]
  public static Trigger SHOOT_FROM_SOURCE;

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
