package frc.robot.util.misc.Obstacles;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class LineObstacle extends GenericObstacle {

    private final Translation2d start;
    private final Translation2d end;

    private final double length;

    private final Rotation2d inverse;
    private final Rotation2d orthogonal;

    public LineObstacle(double strength, boolean repulsive, Translation2d start, Translation2d end) {
        super(strength, repulsive);
        this.start = start;
        this.end = end;

        this.length = start.getDistance(end);

        this.inverse = new Rotation2d(start.getX() - end.getX(), start.getY() - end.getY());
        this.orthogonal = inverse.rotateBy(Rotation2d.fromDegrees(90));
    }

    @Override
    public netForce applyForce(double x, double y, Translation2d goal, netForce currentForce) {
        double startDistx = x - start.getX();
        double startDisty = y - start.getY();

        double projx = startDistx * inverse.getCos() - startDisty * inverse.getSin();
        double projy = startDistx * inverse.getSin() + startDisty * inverse.getCos();

        if(projx > 0.0 && projx < length) {
            double magnitude = Math.copySign(getForceMagnitude(projy), projy);
            return new netForce(currentForce.x() + magnitude*orthogonal.getCos(), currentForce.y() + magnitude*orthogonal.getSin());
        } else {
            Translation2d closest = projx <= 0.0 ? start : end;

            double dx = x - closest.getX();
            double dy = y - closest.getY();

            double dnorm = Math.max(1e-6, Math.hypot(dx, dy));
            double magnitude = getForceMagnitude(dnorm);

            return new netForce(currentForce.x() + magnitude * (dx / dnorm), currentForce.y() + magnitude * (dy / dnorm));
        }
    }
}