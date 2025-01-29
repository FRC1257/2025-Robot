package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

public class VisionConstants {
  // public static final String[] camNames = {"Front_Camera", "Left_Camera", "Right_Camera"};
  public static final String[] camNames = {"Front_Camera"};
  public static final int numCameras = camNames.length;

  // Cam mounted facing forward, half a meter forward of center, half a meter up
  // from center.
  /*public static final Transform3d[] camsRobotToCam = {
    new Transform3d(
        new Translation3d(
            Units.inchesToMeters(-9), Units.inchesToMeters(7), Units.inchesToMeters(10)),
        new Rotation3d(
            0, Rotation2d.fromDegrees(30).getRadians(), Rotation2d.fromDegrees(180).getRadians())),
    new Transform3d(
        new Translation3d(
            Units.inchesToMeters(-0.25), Units.inchesToMeters(4.5), Units.inchesToMeters(11)),
        new Rotation3d(
            0,
            Rotation2d.fromDegrees(90 - 61.90).getRadians(),
            Rotation2d.fromDegrees(90).getRadians())), // maybe need to change
    new Transform3d(
        new Translation3d(
            Units.inchesToMeters(-0.25), Units.inchesToMeters(-4.5), Units.inchesToMeters(11)),
        new Rotation3d(
            0,
            Rotation2d.fromDegrees(90 - 61.90).getRadians(),
            Rotation2d.fromDegrees(-90).getRadians())) // maybe need to change
  };*/
  public static final Transform3d[] camsRobotToCam = {
    new Transform3d(
        new Translation3d(
            Units.inchesToMeters(-9), Units.inchesToMeters(7), Units.inchesToMeters(10)),
        new Rotation3d(
            0, Rotation2d.fromDegrees(30).getRadians(), Rotation2d.fromDegrees(0).getRadians()))
  };

  // The layout of the AprilTags on the field
  public static final AprilTagFieldLayout kTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

  public static final double AMBIGUITY_THRESHOLD = 0.5;
  public static final double MAX_DISTANCE = 4; // meters

  // The standard deviations of our vision estimated poses, which affect
  // correction rate
  // (Fake values. Experiment and determine estimation noise on an actual robot.)
  public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(2, 2, 8);
  public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.2, 0.2, 1);

  public static Transform3d getSimVersion(Transform3d real) {
    return new Transform3d(real.getTranslation(), new Rotation3d(0, 0, real.getRotation().getZ()));
  }
}
