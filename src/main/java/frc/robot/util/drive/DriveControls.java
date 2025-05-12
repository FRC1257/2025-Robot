package frc.robot.util.drive;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Drivers;
import frc.robot.Constants.Operators;
import frc.robot.util.drive.CommandSnailController.DPad;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class DriveControls {
  // Controllers
  public static final CommandSnailController driver = new CommandSnailController(0);
  public static final CommandSnailController operator = new CommandSnailController(1);

  // Useful for things that don't need to be triggered
  private static final Trigger EMPTY_TRIGGER = new Trigger(() -> false);
  private static final DoubleSupplier EMPTY_DOUBLE_SUPPLIER = () -> 0.0;

  // Drive controls
  public static DoubleSupplier DRIVE_FORWARD = EMPTY_DOUBLE_SUPPLIER;
  public static DoubleSupplier DRIVE_STRAFE = EMPTY_DOUBLE_SUPPLIER;
  public static DoubleSupplier DRIVE_ROTATE = EMPTY_DOUBLE_SUPPLIER;
  public static Trigger DRIVE_SLOW = EMPTY_TRIGGER;
  public static Trigger DRIVE_STOP = EMPTY_TRIGGER;
  public static Trigger DRIVE_ROBOT_RELATIVE = EMPTY_TRIGGER;

  public static Trigger TOGGLE_REEF_POSITION_UP = EMPTY_TRIGGER;
  public static Trigger TOGGLE_REEF_POSITION_DOWN = EMPTY_TRIGGER;
  public static Trigger DRIVE_TO_REEF = EMPTY_TRIGGER;

  public static Trigger JOYSTICK_REEF_POINT = EMPTY_TRIGGER;
  public static Trigger JOYSTICK_STATION_POINT = EMPTY_TRIGGER;
  public static Trigger JOYSTICK_PROCESSOR_POINT = EMPTY_TRIGGER;

  public static Trigger ALIGN_REEF = EMPTY_TRIGGER;
  public static Trigger ALIGN_REEF_LEFT = EMPTY_TRIGGER;
  public static Trigger ALIGN_REEF_RIGHT = EMPTY_TRIGGER;
  public static Trigger ALIGN_STATION = EMPTY_TRIGGER;
  public static Trigger ALIGN_REEF_CENTER = EMPTY_TRIGGER;

  // Algae Pivot Controls
  public static DoubleSupplier ALGAE_PIVOT_SPEED = EMPTY_DOUBLE_SUPPLIER;
  public static Trigger ALGAE_PIVOT_STOW = EMPTY_TRIGGER;
  public static Trigger ALGAE_PIVOT_DOWN = EMPTY_TRIGGER;
  public static Trigger ALGAE_PIVOT_PROCESSOR = EMPTY_TRIGGER;

  // Coral pivot controls
  public static DoubleSupplier CORAL_PIVOT_SPEED = EMPTY_DOUBLE_SUPPLIER;
  public static Trigger CORAL_PIVOT_L1 = EMPTY_TRIGGER;
  public static Trigger CORAL_PIVOT_L2_L3 = EMPTY_TRIGGER;
  public static Trigger CORAL_PIVOT_STOW = EMPTY_TRIGGER;
  public static Trigger CORAL_PIVOT_STATION = EMPTY_TRIGGER;

  // Drive Turns
  public static Trigger TURN_90 = EMPTY_TRIGGER;
  public static Trigger TURN_180 = EMPTY_TRIGGER;

  // Rumble Controls
  public static Trigger TIMED_RUMBLE = EMPTY_TRIGGER;
  public static Trigger INTAKE_RUMBLE = EMPTY_TRIGGER;

  // Potential Hail Marry Program [Suggested by Owen]
  public static Trigger SHOOT_FROM_SOURCE = EMPTY_TRIGGER;

  // Algae Intake Controls
  public static Trigger INTAKE_ALGAE = EMPTY_TRIGGER;
  public static Trigger EJECT_ALGAE = EMPTY_TRIGGER;

  // Coral Intake Controls
  public static Trigger INTAKE_CORAL = EMPTY_TRIGGER;
  public static Trigger EJECT_CORAL = EMPTY_TRIGGER;

  // Elevator Controls
  public static DoubleSupplier ELEVATOR_SPEED = EMPTY_DOUBLE_SUPPLIER;
  public static Trigger ELEVATOR_L1 = EMPTY_TRIGGER;
  public static Trigger ELEVATOR_L2 = EMPTY_TRIGGER;
  public static Trigger ELEVATOR_L3 = EMPTY_TRIGGER;
  public static Trigger ELEVATOR_DOWN = EMPTY_TRIGGER;
  public static Trigger ELEVATOR_STATION = EMPTY_TRIGGER;

  // Combined elevator and coral pivot controls
  public static Trigger COMBINED_L1 = EMPTY_TRIGGER;
  public static Trigger COMBINED_L2 = EMPTY_TRIGGER;
  public static Trigger COMBINED_L3 = EMPTY_TRIGGER;
  public static Trigger COMBINED_STATION = EMPTY_TRIGGER;
  public static Trigger COMBINED_STOW = EMPTY_TRIGGER;

  // Creates Elastic dropdown menu for Drivers
  public static class DriverChooser {

    private LoggedDashboardChooser<Drivers> driverChooser;

    public DriverChooser() {
      driverChooser = new LoggedDashboardChooser<>("Driver Selection");
      driverChooser.addDefaultOption("Maddie", Drivers.MADDIE);
      driverChooser.addOption("Michael", Drivers.MICHAEL);
      driverChooser.addOption("Gabe", Drivers.GABE);
      driverChooser.addOption("Programmers", Drivers.PROGRAMMERS);
    }

    public Drivers getDriver() {
      Drivers driver = driverChooser.get();
      if (driver == null) {
        return Drivers.MADDIE;
      }
      return driver;
    }
  }

  // Creates Elastic dropdown menu for Operators
  public static class OperatorChooser {

    private LoggedDashboardChooser<Operators> operatorChooser;

    public OperatorChooser() {
      operatorChooser = new LoggedDashboardChooser<>("Operator Selection");
      operatorChooser.addDefaultOption("Kevin", Operators.KEVIN);
      operatorChooser.addOption("Arboria", Operators.ARBORIA);
      operatorChooser.addOption("Antonios", Operators.ANTONIOS);
      operatorChooser.addOption("Programmers", Operators.PROGRAMMERS);
    }

    public Operators getOperator() {
      Operators operator = operatorChooser.get();
      if (operator == null) {
        return Operators.KEVIN;
      }
      return operator;
    }
  }

  // Instantiate the choosers
  private static final DriverChooser driverChooser = new DriverChooser();
  private static final OperatorChooser operatorChooser = new OperatorChooser();

  // Setup the controls
  public static void configureControls() {
    switch (driverChooser.getDriver()) {
      case MADDIE:
      case MICHAEL:
        DRIVE_FORWARD = () -> -driver.getLeftY();
        DRIVE_STRAFE = () -> -driver.getLeftX();
        DRIVE_ROTATE = () -> -driver.getRightX();
        DRIVE_STOP = driver.x();
        DRIVE_SLOW = driver.rightTrigger();
        DRIVE_ROBOT_RELATIVE = EMPTY_TRIGGER;

        // TURN_90 = driver.y();
        TURN_180 = driver.start();

        TOGGLE_REEF_POSITION_UP = driver.getDPad(DPad.UP);
        TOGGLE_REEF_POSITION_DOWN = driver.getDPad(DPad.DOWN);
        DRIVE_TO_REEF = driver.getDPad(DPad.LEFT);

        JOYSTICK_REEF_POINT = driver.b();
        JOYSTICK_STATION_POINT = driver.y();
        JOYSTICK_PROCESSOR_POINT = EMPTY_TRIGGER;

        ALIGN_REEF_LEFT = driver.leftBumper();
        ALIGN_REEF_RIGHT = driver.rightBumper();
        ALIGN_STATION = driver.leftTrigger();
        ALIGN_REEF_CENTER = driver.rightTrigger();
        break;
      case GABE:
      case PROGRAMMERS:
      default:
        DRIVE_FORWARD = () -> -driver.getLeftY();
        DRIVE_STRAFE = () -> -driver.getLeftX();
        DRIVE_ROTATE = () -> -driver.getRightX();
        DRIVE_STOP = driver.x();
        DRIVE_SLOW = driver.rightBumper();
        DRIVE_ROBOT_RELATIVE = EMPTY_TRIGGER;

        // TURN_90 = driver.y();
        TURN_180 = driver.start();

        TOGGLE_REEF_POSITION_UP = driver.getDPad(DPad.UP);
        TOGGLE_REEF_POSITION_DOWN = driver.getDPad(DPad.DOWN);
        DRIVE_TO_REEF = EMPTY_TRIGGER;

        JOYSTICK_REEF_POINT = driver.b();
        JOYSTICK_STATION_POINT = driver.y();
        JOYSTICK_PROCESSOR_POINT = EMPTY_TRIGGER;

        ALIGN_REEF = driver.leftBumper();
        ALIGN_STATION = driver.leftTrigger();
        break;
    }

    switch (operatorChooser.getOperator()) {
      case ARBORIA:
        COMBINED_L1 = operator.getDPad(DPad.LEFT);
        COMBINED_L2 = operator.getDPad(DPad.RIGHT);
        COMBINED_L3 = operator.getDPad(DPad.UP);
        COMBINED_STOW = operator.getDPad(DPad.DOWN);
        COMBINED_STATION = operator.a();

        ELEVATOR_SPEED = () -> -operator.getLeftYD();
        CORAL_PIVOT_SPEED = () -> -operator.getRightYD() * 0.1;

        ALGAE_PIVOT_STOW = operator.x();
        ALGAE_PIVOT_PROCESSOR = operator.y();
        ALGAE_PIVOT_DOWN = operator.b();

        INTAKE_CORAL = operator.rightBumper();
        EJECT_CORAL = operator.leftBumper();

        INTAKE_ALGAE = operator.rightTrigger();
        EJECT_ALGAE = operator.leftTrigger();
        break;
      case KEVIN:
        COMBINED_L1 = operator.getDPad(DPad.LEFT);
        COMBINED_L2 = operator.getDPad(DPad.RIGHT);
        COMBINED_L3 = operator.getDPad(DPad.UP);
        COMBINED_STOW = operator.getDPad(DPad.DOWN);
        COMBINED_STATION = operator.a();

        ELEVATOR_SPEED = () -> -operator.getLeftYD();
        CORAL_PIVOT_SPEED = () -> -operator.getRightYD() * 0.1;

        ALGAE_PIVOT_STOW = operator.x();
        ALGAE_PIVOT_PROCESSOR = operator.y();
        ALGAE_PIVOT_DOWN = operator.b();

        INTAKE_CORAL = operator.rightBumper();
        EJECT_CORAL = operator.leftBumper();

        INTAKE_ALGAE = operator.rightTrigger();
        EJECT_ALGAE = operator.leftTrigger();
        break;

      case ANTONIOS:
        COMBINED_L1 = operator.getDPad(DPad.LEFT);
        COMBINED_L2 = operator.getDPad(DPad.RIGHT);
        COMBINED_L3 = operator.getDPad(DPad.UP);
        COMBINED_STOW = operator.getDPad(DPad.DOWN);
        COMBINED_STATION = operator.a();

        // ELEVATOR_DOWN = operator.getDPad(DPad.DOWN);
        // ELEVATOR_L1 = operator.getDPad(DPad.LEFT);
        // ELEVATOR_L2 = operator.getDPad(DPad.RIGHT);
        // ELEVATOR_L3 = operator.getDPad(DPad.UP);

        // CORAL_PIVOT_STATION = operator.a();

        // CORAL_PIVOT_STOW = operator.getDPad(DPad.DOWN);
        // CORAL_PIVOT_STATION = operator.getDPad(DPad.UP);

        ELEVATOR_SPEED = () -> -operator.getLeftYD();
        CORAL_PIVOT_SPEED = () -> -operator.getRightYD() * 0.1;

        ALGAE_PIVOT_STOW = operator.x();
        ALGAE_PIVOT_PROCESSOR = operator.y();
        ALGAE_PIVOT_DOWN = operator.b();
        // ALGAE_PIVOT_SPEED = () -> -operator.getRightYD() * 0.1;

        INTAKE_CORAL = operator.leftBumper();
        EJECT_CORAL = operator.rightBumper();

        INTAKE_ALGAE = operator.leftTrigger();
        EJECT_ALGAE = operator.rightTrigger();
        break;
      case PROGRAMMERS:
      default:
        // Operator controls
        ALGAE_PIVOT_SPEED = EMPTY_DOUBLE_SUPPLIER;
        ALGAE_PIVOT_DOWN = EMPTY_TRIGGER;
        ALGAE_PIVOT_STOW = EMPTY_TRIGGER;
        ALGAE_PIVOT_PROCESSOR = EMPTY_TRIGGER;

        INTAKE_CORAL = EMPTY_TRIGGER;
        EJECT_CORAL = EMPTY_TRIGGER;
        EJECT_CORAL = EMPTY_TRIGGER;

        CORAL_PIVOT_SPEED = () -> -operator.getLeftYD();
        CORAL_PIVOT_L1 = operator.a();
        CORAL_PIVOT_L2_L3 = operator.b();
        CORAL_PIVOT_STOW = operator.x();
        CORAL_PIVOT_STATION = operator.y();

        INTAKE_ALGAE = EMPTY_TRIGGER;
        EJECT_ALGAE = EMPTY_TRIGGER;
        EJECT_ALGAE = EMPTY_TRIGGER;

        // ELEVATOR_SPEED = () -> -operator.getLeftYD();
        // ELEVATOR_L1 = operator.getDPad(DPad.LEFT);
        ELEVATOR_L2 = operator.getDPad(DPad.RIGHT);
        ELEVATOR_L3 = operator.getDPad(DPad.UP);
        // ELEVATOR_DOWN = operator.b();
        ELEVATOR_STATION = operator.getDPad(DPad.DOWN);
        break;
    }
  }

  public static void updateDriverAndOperator() {
    driverChooser.getDriver();
    operatorChooser.getOperator();
    configureControls();
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
