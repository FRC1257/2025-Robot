package frc.robot.util.autonomous;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.FieldConstants;

public class CustomAutoChooser {
    public static enum Positions {
        S1, S2, S3,
        C1, C2,
        A1, A2, A3,
        R1, R2, R3, R4, R5, R6, R7, R8, R9, R10, R11, R12,
        P
    }

    private LoggedDashboardChooser<Positions> startChooser;
    private LoggedDashboardChooser<Positions>[] positionChoosers = new LoggedDashboardChooser[5];
    

    public CustomAutoChooser() {
        
        startChooser = new LoggedDashboardChooser<>("Starting Position ");
        startChooser.addDefaultOption("Starting Position 1", Positions.S1);
        startChooser.addOption("Starting Position 2", Positions.S2);
        startChooser.addOption("Starting Position 3", Positions.S3);

        for(int i = 0; i < positionChoosers.length; i++) {
            positionChoosers[i] = new LoggedDashboardChooser<>("Position " + (i + 1));
            positionChoosers[i].addDefaultOption("R1", Positions.R1);

            for(Positions position : Positions.values()) {
                positionChoosers[i].addOption(position.toString(), position);
            }
        }


    }

    public Pose2d getStartPosition() {
        return startChooser.get();
    }

}
