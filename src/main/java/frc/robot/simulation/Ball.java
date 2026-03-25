package frc.robot.simulation;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;

public class Ball {
    public double x, y, z;
    public double vx, vy, vz;

    private double radius;
    private boolean ignoreRobot = false;

    private static final double friction = 0.985;
    private static final double gravity  = -9.8 * 0.02 * 0.02;

    public Ball(Pose3d pos, double radius) {
        this.x = pos.getX();
        this.y = pos.getY();
        this.z = pos.getZ();

        this.vx = 0;
        this.vy = 0;
        this.vz = 0;

        this.radius = radius;
    }

    public double getRadius() { return radius; }

    public Pose3d getPosition() {
        return new Pose3d(x, y, z, new Rotation3d());
    }

    public void setPosition(Pose3d p) {
        x = p.getX();
        y = p.getY();
        z = p.getZ();
    }

    public void setPosition(double x, double y, double z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    public Translation3d getVelocity() {
        return new Translation3d(vx, vy, vz);
    }

    public void setVelocity(Translation3d v) {
        vx = v.getX();
        vy = v.getY();
        vz = v.getZ();
    }

    public void setVelocity(double vx, double vy, double vz) {
        this.vx = vx; this.vy = vy; this.vz = vz;
    }

    public void addVelocity(double dx, double dy, double dz) {
        vx += dx;
        vy += dy;
        vz += dz;
    }

    public boolean getIgnoreRobot() {
        return ignoreRobot;
    }

    public void setIgnoreRobot(boolean ignore) {
        ignoreRobot = ignore;
    }

    public void tick() {
        vz += gravity;

        x += vx;
        y += vy;
        z += vz;

        if (z <= radius + 1e-4) { // just in case there is a SMALLLLLLL precision error
            vx *= friction;
            vy *= friction;
        }
    }
}
