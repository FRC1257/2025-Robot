package frc.robot.util.misc.Obstacles;

import edu.wpi.first.math.geometry.Translation2d;

public abstract class GenericObstacle {

    private final double strength;
    private final boolean repulsive;

    /**
     * constructs a genericObstacle
     * @param strength The strength of the obstacle. Higher values mean a stronger force.
     * @param repulsive if the obstacle supplies repulsive potential. if {@code false}, the obstacle
     *                  supplies attractive potential
     */

    public GenericObstacle(double strength, boolean repulsive) {
        this.strength = strength;
        this.repulsive = repulsive;
    }

    /**
     * Applies the force of the obstacle to the provided accumulator based upon the robots current position and goal
     * @param x The current x position of the robot on the field
     * @param y The current y position of the robot on the field
     * @param goal The goal position of the robot on the field
     * @param force The current force accumulator of the robot to be modified
     * @return The modified force accumulator
     */
    public abstract netForce applyForce(double x, double y, Translation2d goal, netForce force);

    /**
     * converts the distance from an obstacle to a force magnitude
     * @param dist The distance from the robot to the obstacle
     * @return The force magnitude to be applied to the robot
     */
    protected final double getForceMagnitude(double dist) {
        double dist_2 = Math.max(1e-6, dist * dist);
        return (strength / dist_2) * (repulsive ? 1.0 : -1.0);
    }

}

record netForce(double x, double y) {}
