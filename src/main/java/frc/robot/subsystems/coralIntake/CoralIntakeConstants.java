package frc.robot.subsystems.coralIntake;

public class CoralIntakeConstants {
  public static class CoralIntakeSimConstants {
    public static final int kMotorPort = 0;

    public static final double kCoralIntakeP = 0.001;
    public static final double kCoralIntakeI = 0.0;
    public static final double kCoralIntakeD = 0.0;

    public static final double kCoralIntakeGearing = 1.2;
    public static final double kCoralIntakeDrumRadius = 0.03;
    public static final double kCarriageMass = 0.15; // Mass in Kg
    public static final double kMomentOfInertia =
        0.5
            * kCarriageMass
            * kCoralIntakeDrumRadius
            * kCoralIntakeDrumRadius; // Moment of inertia represents how resistant to force
    // something is
  }

  public static class CoralIntakePhysicalConstants {
    public static final double kCoralIntakeP = 0.001;
    public static final double kCoralIntakeI = 0.0;
    public static final double kCoralIntakeD = 0.0;
  }

  public static final double CORAL_INTAKE_IN_VOLTAGE = 9.2;
  public static final double CORAL_INTAKE_WEAK_IN_VOLTAGE = 4.5;
  public static final double CORAL_INTAKE_OUT_VOLTAGE = -9;
  public static final double CORAL_INTAKE_TOLERANCE = 1;
}
