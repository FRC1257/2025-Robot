package frc.robot.util.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class CustomAutoChooser {
  public static enum StartPositions {
    s1,
    s2,
    s3
  }

  public static enum NextPositions {
    c1,
    c2,
    a1,
    a2,
    a3,
    r1,
    r2,
    r3,
    r4,
    r5,
    r6,
    r7,
    r8,
    r9,
    r10,
    r11,
    r12,
    p,
    NONE
  }

  private LoggedDashboardChooser<StartPositions> startChooser;
  private LoggedDashboardChooser<NextPositions>[] positionChoosers = new LoggedDashboardChooser[5];

  private Drive drive;

  public CustomAutoChooser(Drive drive) {
    this.drive = drive;

    startChooser = new LoggedDashboardChooser<>("Starting Position ");
    startChooser.addDefaultOption("Starting Position 1", StartPositions.s1);
    startChooser.addOption("Starting Position 2", StartPositions.s2);
    startChooser.addOption("Starting Position 3", StartPositions.s3);

    for (int i = 0; i < positionChoosers.length; i++) {
      positionChoosers[i] = new LoggedDashboardChooser<>("Position " + (i + 1));
      positionChoosers[i].addDefaultOption("", NextPositions.NONE);

      for (NextPositions position : NextPositions.values()) {
        if (position == NextPositions.NONE) continue;
        positionChoosers[i].addOption(position.toString(), position);
      }
    }
  }

  public Command getAutoCommand() {
    StartPositions startPos = startChooser.get();
    NextPositions nextPoses[] = new NextPositions[positionChoosers.length];
    SequentialCommandGroup commandGroup = new SequentialCommandGroup();

    Pose2d startPose2d;
    switch (startPos) {
      case s1:
        startPose2d = FieldConstants.StartingPositions.startPos1;
        break;
      case s2:
        startPose2d = FieldConstants.StartingPositions.startPos2;
        break;
      case s3:
        startPose2d = FieldConstants.StartingPositions.startPos3;
        break;
      default:
        startPose2d = new Pose2d();
        break;
    }

    commandGroup.addCommands(
        new InstantCommand(
            () -> {
              drive.setPose(startPose2d);
            },
            drive));

    nextPoses[0] = positionChoosers[0].get();
    if (nextPoses[0] != NextPositions.NONE)
      commandGroup.addCommands(
          drive.followPathFileCommand(startPos.toString() + "-" + nextPoses[0].toString()));

    for (int i = 1; i < positionChoosers.length; i++) {
      nextPoses[i] = positionChoosers[i].get();
      if (nextPoses[i] == NextPositions.NONE) continue;
      commandGroup.addCommands(
          drive.followPathFileCommand(nextPoses[i - 1].toString() + "-" + nextPoses[i].toString()));
    }

    return commandGroup;
  }
}
