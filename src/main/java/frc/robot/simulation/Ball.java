package frc.robot.simulation;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;

// commented it a lot if ur curious

public class Ball {
    private Pose3d pos;
    private double radius;
    private Translation3d velocity;

    private static final double friction = 0.985;
    private static final double gravity  = -9.8 * 0.02 * 0.02; // time squared

    private boolean ignoreRobot = false;

    public Ball(Pose3d pos_, double radius_) {
        pos = pos_;
        radius = radius_;
        velocity = new Translation3d(0, 0, 0);
    }

    public double getRadius() { return radius; }
    
    public Pose3d getPosition() { return pos; }
    public void setPosition(Pose3d pos_) { pos = pos_; }

    public Translation3d getVelocity() { return velocity; }
    public void setVelocity(Translation3d velocity_) { velocity = velocity_; }

    public void setIgnoreRobot(boolean ignore) {
        ignoreRobot = ignore;
    }

    public boolean getIgnoreRobot() {
        return ignoreRobot;
    }

    public void addVelocity(double dx, double dy, double dz) {
        velocity = new Translation3d(
            velocity.getX() + dx,
            velocity.getY() + dy,
            velocity.getZ() + dz
        );
    }

    public void tick() {
        // called every 20 milliseconds
        // does gravity and updates position

        addVelocity(0, 0, gravity);

        pos = new Pose3d(
            pos.getX() + velocity.getX(),
            pos.getY() + velocity.getY(),
            pos.getZ() + velocity.getZ(),
            pos.getRotation()
        );

        // apply friction when the ball is on the ground
        // the 1e-4 is for any imprecision, basically safety
        if (pos.getZ() <= radius + 1e-4) {
            velocity = new Translation3d(
                velocity.getX() * friction,
                velocity.getY() * friction,
                velocity.getZ()
            );
        }
    }
}
