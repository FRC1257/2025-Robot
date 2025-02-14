package frc.robot.subsystems.algaeIntake;

public class AlgaeIntakeConstants {
  public static class AlgaeIntakeSimConstants {
    public static final double kAlgaeIntakeP = 0.001;
    public static final double kAlgaeIntakeI = 0.0;
    public static final double kAlgaeIntakeD = 0.0;

    public static final double kAlgaeIntakeGearing = 1.2;
    public static final double kAlgaeIntakeDrumRadius = 0.03;
    public static final double kCarriageMass = 0.15; // Mass in Kg
    public static final double kMomentOfInertia =
        0.5
            * kCarriageMass
            * kAlgaeIntakeDrumRadius
            * kAlgaeIntakeDrumRadius; // Moment of inertia represents how resistant to force
    // something is
  }

  public static final int ALGAE_INTAKE_MOTOR_ID = 0;

  public static final double ALGAE_INTAKE_IN_VOLTAGE = 9.2;
  public static final double ALGAE_INTAKE_WEAK_IN_VOLTAGE = 4.5;
  public static final double ALGAE_INTAKE_OUT_VOLTAGE = -9;
  public static final double ALGAE_INTAKE_TOLERANCE = 1;
}
