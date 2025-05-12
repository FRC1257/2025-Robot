package frc.robot.util.autonomous;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.util.misc.Obstacles.GenericObstacle;

public final class PAPFController {

    private final GenericObstacle[] obstacles;

    private double resolution;
    private double horizon;
    private boolean savePrediction;

    private final List<Translation2d> predictions; 
    private Pose2d setpoint = Pose2d.kZero;
    private Pose2d goal = Pose2d.kZero;

    public PAPFController(double resolution, double horizon, boolean savePrediction, GenericObstacle[] obstacles) {

        this.resolution = resolution;
        this.horizon = horizon;
        this.savePrediction = savePrediction;
        this.obstacles = obstacles;

        this.predictions = new ArrayList<>((int) Math.ceil(horizon / resolution));
    }

    public Translation2d getSetpoint() {
        return setpoint.getTranslation();
    }

    public Translation2d getGoal() {
        return goal.getTranslation();
    }

    public List<Translation2d> getPredictions() {
        return Collections.unmodifiableList(predictions);
    }

}