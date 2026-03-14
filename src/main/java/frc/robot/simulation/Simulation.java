package frc.robot.simulation;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

// commented it a lot if ur curious

public class Simulation {
    public static double ballRadius = 0.075;

    public static double length = 17.37; // close to, but not actual length
    public static double width = 7.99; // close to, but not actual width

    public static int maxStorage = 40; // idk

    public static double shootHeightError = 1.5;
    public static double minShootAngle = 0.0;
    public static double maxShootAngle = 80.0; // in degrees

    private static StructArrayPublisher<Pose3d> ballPublisher;

    private static StructArrayPublisher<Pose3d> vertexPublisher;

    private static StructArrayPublisher<Pose3d> intakePublisher;

    private static List<Ball> balls = new ArrayList<>();
    private static List<Ball> storage = new ArrayList<>();

    private static double shootAngle = 0.0; // above the xy plane
    private static double shootSpeed = 0.0; // speed, not velocity since velocity is a vector

    public static final Translation3d shooterPosition = new Translation3d(0.0, 0.0, 0.0);

    public static void init() {
        // making balls in the center

        balls.clear();
        storage.clear();

        double centerX = (length / 2.0) - 0.4325;
        double centerY = (width / 2.0) + 0.075;

        int cols = 12;
        int rows = 30;
        double spacing = 0.2;

        double gridWidthX = (cols - 1) * spacing;
        double gridHeightY = (rows - 1) * spacing;

        for (int i = 0; i < (cols * rows); i++) {
            int col = i % cols;
            int row = i / cols;

            double x = centerX - (gridWidthX / 2.0) + (col * spacing);
            double y = centerY - (gridHeightY / 2.0) + (row * spacing);
            double z = ballRadius;

            balls.add(new Ball(
                new Pose3d(x, y, z, new Rotation3d()), 
                ballRadius
            ));
        }

        // so u can see stuff in advantage scope

        ballPublisher = NetworkTableInstance.getDefault().getStructArrayTopic("/Field/Ball Positions", Pose3d.struct).publish();
        vertexPublisher = NetworkTableInstance.getDefault().getStructArrayTopic("/Field/Robot Vertices", Pose3d.struct).publish();
        intakePublisher = NetworkTableInstance.getDefault().getStructArrayTopic("/Field/Robot Intake", Pose3d.struct).publish();
    }

    public static void updateVertexPositionsAdvantageScope(Translation3d[] points, Translation3d[] intakePoints, Pose3d origin) {
        // makes these local positions into world positions and gives it to advantage scope   
        
        Pose3d[] worldVertices = new Pose3d[points.length];
        
        for (int i = 0; i < points.length; i++)
            worldVertices[i] = transformToWorld(points[i], origin);
        
        vertexPublisher.set(worldVertices);

        // same for this

        Pose3d[] worldVerticesIntake = new Pose3d[intakePoints.length];

        for (int i = 0; i < intakePoints.length; i++)
            worldVerticesIntake[i] = transformToWorld(intakePoints[i], origin);
        
        intakePublisher.set(worldVerticesIntake);
    }

    public static void shootBall(Pose3d robotPose) {
        if (storage.isEmpty()) return;

        Ball ball = storage.remove(0);
        
        Pose3d spawnPose = transformToWorld(shooterPosition, robotPose);

        // yaw is the heading (rotation on the z axis)
        double yaw = robotPose.getRotation().getZ() + (Math.PI / 2.0);
        double elevationRadians = Math.toRadians(shootAngle);

        // vxy is the speed along the ground (xy plane) and vy is the speed vertically (z axis)
        double vxy = shootSpeed * Math.cos(elevationRadians);
        double vz  = shootSpeed * Math.sin(elevationRadians);

        // splits speed along the xy plane into speeds correlating to their axis
        double vx = vxy * Math.cos(yaw);
        double vy = vxy * Math.sin(yaw);

        // put the ball at the shooter and let if fly :)
        ball.setPosition(spawnPose);
        ball.setVelocity(new Translation3d(vx, vy, vz));
        ball.setIgnoreRobot(true);

        // finally add the ball background storage to the "world"
        balls.add(ball);
    }

    public static void updateBalls(Translation3d[] points, Translation3d[] intakePoints, Pose3d origin) {
        // iterator for future removal to be easy
        Iterator<Ball> iterator = balls.iterator();

        while (iterator.hasNext()) {
            Ball ball = iterator.next();
            ball.tick(); // update balls physics (like minecraft tick shit)

            // if were ignoring, then stop messing with it
            if (ball.getIgnoreRobot()) {
                if (getPushVector(ball, points, origin) == null) {
                    ball.setIgnoreRobot(false);
                }
            }

            // if it's in the "world" and were able to fit in storage, plus it's in that collisions zone, we add it, and remove it from the "world"
            if (!ball.getIgnoreRobot() && storage.size() < maxStorage && isBallInIntake(ball, intakePoints, origin)) {
                storage.add(ball);
                iterator.remove();
                continue; 
            }

            // make sure it's in the "world"
            if (!ball.getIgnoreRobot()) {
                Translation3d push = getPushVector(ball, points, origin);
                
                if (push != null) {
                    // in the name, just pushing it

                    Pose3d pos = ball.getPosition();

                    ball.setPosition(new Pose3d(
                        pos.getX() + push.getX(),
                        pos.getY() + push.getY(),
                        pos.getZ() + push.getZ(),
                        pos.getRotation()
                    ));

                    // adding a velocity to push it away
                    double pushMagnitude = Math.sqrt(push.getX()*push.getX() + push.getY()*push.getY() + push.getZ()*push.getZ());
                    if (pushMagnitude > 1e-9) {
                        double px = push.getX() / pushMagnitude;
                        double py = push.getY() / pushMagnitude;
                        double pz = push.getZ() / pushMagnitude;
                        double impulse = Math.min(pushMagnitude, 0.02); // cap it, not the best way, but later should prob be a variable
                        ball.addVelocity(px * impulse, py * impulse, pz * impulse);
                    }
                }
            }

            // keep the ball inside the field
            handleWallCollisions(ball);
        }

        // collisions between the balls
        handleBallCollisions();
    }

    private static void handleBallCollisions() {
        for (int i = 0; i < balls.size(); i++) {
            for (int j = i + 1; j < balls.size(); j++) {
                Ball a = balls.get(i);
                Ball b = balls.get(j);

                Pose3d aPos = a.getPosition();
                Pose3d bPos = b.getPosition();

                // difference in positions
                double dx = bPos.getX() - aPos.getX();
                double dy = bPos.getY() - aPos.getY();
                double dz = bPos.getZ() - aPos.getZ();
                double distance = Math.sqrt(dx*dx + dy*dy + dz*dz);
                double minDistance = a.getRadius() + b.getRadius();

                if (distance < minDistance && distance > 1e-9) {
                    // normalize aka getting the directions
                    double nx = dx / distance;
                    double ny = dy / distance;
                    double nz = dz / distance;

                    double overlap = (minDistance - distance) / 2.0;

                    // using overlap and pushing so they don't touch
                    a.setPosition(new Pose3d(aPos.getX() - nx * overlap, aPos.getY() - ny * overlap, aPos.getZ() - nz * overlap, aPos.getRotation()));
                    b.setPosition(new Pose3d(bPos.getX() + nx * overlap, bPos.getY() + ny * overlap, bPos.getZ() + nz * overlap, bPos.getRotation()));

                    Translation3d va = a.getVelocity();
                    Translation3d vb = b.getVelocity();

                    // get relative velocity between the balls
                    double dvx = va.getX() - vb.getX();
                    double dvy = va.getY() - vb.getY();
                    double dvz = va.getZ() - vb.getZ();

                    // project the relative velocity onto the collision normal
                    // so if the dot is greater than 0, the balls are moving towards each other
                    // if not (less than or equal to 0), they already are separating
                    double dot = dvx * nx + dvy * ny + dvz * nz;

                    // so if they are moving towards each other aka dot greater than 0, we do smth
                    if (dot > 0) {
                        double impulse = dot * 0.8;

                        // that smth being providing an impulse
                        a.setVelocity(new Translation3d(va.getX() - impulse * nx, va.getY() - impulse * ny, va.getZ() - impulse * nz));
                        b.setVelocity(new Translation3d(vb.getX() + impulse * nx, vb.getY() + impulse * ny, vb.getZ() + impulse * nz));
                    }
                }
            }
        }
    }

    private static boolean isBallInIntake(Ball ball, Translation3d[] intakePoints, Pose3d origin) {
        Pose3d ballPos = ball.getPosition();
        double yaw = origin.getRotation().getZ();

        // save it for later
        double cosY = Math.cos(-yaw);
        double sinY = Math.sin(-yaw);
        
        // difference between the ball and the origin
        double dx = ballPos.getX() - origin.getX();
        double dy = ballPos.getY() - origin.getY();
        double dz = ballPos.getZ() - origin.getZ();

        // 2d rotation on the xy plane
        // puts it in robot local space
        double lx = cosY * dx - sinY * dy;
        double ly = sinY * dx + cosY * dy;
        double lz = dz;

        double minX = Double.POSITIVE_INFINITY;
        double maxX = Double.NEGATIVE_INFINITY;

        double minY = Double.POSITIVE_INFINITY;
        double maxY = Double.NEGATIVE_INFINITY;

        double minZ = Double.POSITIVE_INFINITY;
        double maxZ = Double.NEGATIVE_INFINITY;

        // get the minimum and maximum extends of the intake zone
        for (Translation3d p : intakePoints) {
            minX = Math.min(minX, p.getX()); maxX = Math.max(maxX, p.getX());
            minY = Math.min(minY, p.getY()); maxY = Math.max(maxY, p.getY());
            minZ = Math.min(minZ, p.getZ()); maxZ = Math.max(maxZ, p.getZ());
        }

        double radius = ball.getRadius();

        // expand box by ball radius so it's not just checking for its center but any part of the ball in the zone
        return (lx >= minX - radius && lx <= maxX + radius) && (ly >= minY - radius && ly <= maxY + radius) && (lz >= minZ - radius && lz <= maxZ + radius);
    }

    private static void handleWallCollisions(Ball ball) {
        Pose3d pos = ball.getPosition();
        
        // clamp x and y to make it between the field boundaries
        double newX = MathUtil.clamp(pos.getX(), ballRadius, length - ballRadius);
        double newY = MathUtil.clamp(pos.getY(), ballRadius, width - ballRadius);
        
        // reverses the velocity depending on which wall it hits
        if (newX != pos.getX() || newY != pos.getY()) {
            Translation3d v = ball.getVelocity();

            double vx = (newX != pos.getX()) ? -v.getX() * 0.5 : v.getX();
            double vy = (newY != pos.getY()) ? -v.getY() * 0.5 : v.getY();
            
            ball.setVelocity(new Translation3d(vx, vy, v.getZ()));
            ball.setPosition(new Pose3d(newX, newY, pos.getZ(), pos.getRotation()));
        }

        // if ball below the ground, push it up
        // also it got bouncing :)
        if (pos.getZ() < ballRadius) {
            ball.setPosition(new Pose3d(pos.getX(), pos.getY(), ballRadius, pos.getRotation()));
            Translation3d v = ball.getVelocity();

            // bounce dampens by 40 percent every time, should prob make it a variable
            if (v.getZ() < 0) ball.setVelocity(new Translation3d(v.getX(), v.getY(), -v.getZ() * 0.4));
        }
    }

    private static Translation3d getPushVector(Ball ball, Translation3d[] points, Pose3d origin) {
        Pose3d sphereWorld = ball.getPosition();

        double yaw = origin.getRotation().getZ();
        double cosY = Math.cos(-yaw);
        double sinY = Math.sin(-yaw);

        // difference
        double dx = sphereWorld.getX() - origin.getX();
        double dy = sphereWorld.getY() - origin.getY();
        double dz = sphereWorld.getZ() - origin.getZ();

        // put the ball in the robot local space
        // same trick done in the isBallInIntake method
        double lx = cosY * dx - sinY * dy;
        double ly = sinY * dx + cosY * dy;
        double lz = dz;

        double L = 0;
        double W = 0;
        double H = 0;

        // L, W, H are half extents of the robot box
        for (Translation3d p : points) {
            L = Math.max(L, Math.abs(p.getX()));
            W = Math.max(W, Math.abs(p.getY()));
            H = Math.max(H, Math.abs(p.getZ()));
        }

        double radius = ball.getRadius();

        // checks if ball center is inside (KEYWORD, "center")
        boolean inside = Math.abs(lx) <= L && Math.abs(ly) <= W && Math.abs(lz) <= H;

        if (!inside) {
            // closest point on the box surface to the ball center
            double cx = Math.max(-L, Math.min(L, lx));
            double cy = Math.max(-W, Math.min(W, ly));
            double cz = Math.max(-H, Math.min(H, lz));

            // distance from ball center to that closest point
            double ddx = lx - cx, ddy = ly - cy, ddz = lz - cz;
            double dist = Math.sqrt(ddx*ddx + ddy*ddy + ddz*ddz);
            
            // dont collide if the ball is farther than radius
            if (dist >= radius || dist < 1e-9) return null;

            // just penetration
            double pen = radius - dist;

            // direction of push (from closest point toward ball center) (normalized)
            double nx = ddx/dist, ny = ddy/dist, nz = ddz/dist;
            
            // rotate push direction back to world space
            double wx = cosY * nx - sinY * ny;
            double wy = sinY * nx + cosY * ny;

            // now push it
            return new Translation3d(wx * pen, wy * pen, nz * pen);
        } else {
            // end goal: get nearest face to push the ball out

            double distPX = L - lx;
            double distNX = L + lx;

            double distPY = W - ly;
            double distNY = W + ly;
            double distPZ = H - lz;
            double distNZ = H + lz;

            // find the closest face
            double min = Math.min(Math.min(Math.min(distPX, distNX), Math.min(distPY, distNY)), Math.min(distPZ, distNZ));
            
            double nx = 0;
            double ny = 0;
            double nz = 0;

            // okay, now ACTUALLY get the closest face
            if (min == distPX) nx = 1;
            else if (min == distNX) nx = -1;
            else if (min == distPY) ny =  1;
            else if (min == distNY) ny = -1;
            else if (min == distPZ) nz =  1;
            else nz = -1;

            // penetration -_-
            double pen = min + radius;

            // put that shi back to world space
            double wx = cosY * nx - sinY * ny;
            double wy = sinY * nx + cosY * ny;

            // and push it :)
            return new Translation3d(wx * pen, wy * pen, nz * pen);
        }
    }

    private static Pose3d transformToWorld(Translation3d localPoint, Pose3d origin) {
        double roll = origin.getRotation().getX();
        double pitch = origin.getRotation().getY();
        double yaw = origin.getRotation().getZ();

        double x = localPoint.getX();
        double y = localPoint.getY();
        double z = localPoint.getZ();

        double cosR = Math.cos(roll);
        double sinR = Math.sin(roll);

        double cosP = Math.cos(pitch);
        double sinP = Math.sin(pitch);

        double cosY = Math.cos(yaw);
        double sinY = Math.sin(yaw);
        
        // 3d matrix rotation shi since we consider rotation
        double rotatedX = cosY * cosP * x + (cosY * sinP * sinR - sinY * cosR) * y + (cosY * sinP * cosR + sinY * sinR) * z;
        double rotatedY = sinY * cosP * x + (sinY * sinP * sinR + cosY * cosR) * y + (sinY * sinP * cosR - cosY * sinR) * z;
        double rotatedZ = -sinP * x + cosP * sinR * y + cosP * cosR * z;

        // add world pos to get final pos
        return new Pose3d(
            rotatedX + origin.getX(),
            rotatedY + origin.getY(),
            rotatedZ + origin.getZ(),
            origin.getRotation()
        );
    }

    public static void updateFakeLimelight(Translation3d targetPos, SwerveSubsystem drivebase) {
        Pose2d robotPose2d = drivebase.getPose();
        
        double dx = targetPos.getX() - robotPose2d.getX();
        double dy = targetPos.getY() - robotPose2d.getY();
        
        // ignore height difference
        double horizontalDistance = Math.sqrt(dx * dx + dy * dy);

        double targetAngle = Math.atan2(dy, dx);
        targetAngle += (Math.PI / 2.0); // error fix

        double robotHeading = robotPose2d.getRotation().getRadians();
        
        // get the tx from the difference between angle and heading
        double tx = Math.toDegrees(targetAngle - robotHeading);
        tx = MathUtil.inputModulus(tx, -180, 180);
        
        FakeLimelight.setTX(tx);
        FakeLimelight.setHasTarget(true);


        double gravityTick = 9.8 * 0.02 * 0.02; // time is squared

        double startZ = shooterPosition.getZ();
        double targetZ = targetPos.getZ();

        // max z you'll get (shooting error is for an arc)
        double peakZ = targetZ + shootHeightError;

        // getting the z velocity from the peak
        double hPeak = peakZ - startZ;
        if (hPeak < 0.1) hPeak = 0.1; // no sqrt of negative or tiny number (safety)
        double vz = Math.sqrt(2 * gravityTick * hPeak);

        // time to reach peak
        double t1 = vz / gravityTick;

        // time to fall down from peak
        double hDown = peakZ - targetZ;
        double t2 = Math.sqrt(2 * Math.max(0, hDown) / gravityTick);
        
        double totalTime = t1 + t2;

        // horizontal velocity needed (velocity along the xy plane)
        double vxy = horizontalDistance / totalTime;

        // set shoot speed and angle
        // also, shoot angle is degrees cuz i like it
        shootSpeed = Math.sqrt(vxy * vxy + vz * vz);
        shootAngle = Math.toDegrees(Math.atan2(vz, vxy));
        
        // apply the bounds
        shootAngle = MathUtil.clamp(shootAngle, minShootAngle, maxShootAngle);

        // debugging
        SmartDashboard.putNumber("Sim/Shoot Speed", shootSpeed);
        SmartDashboard.putNumber("Sim/Shoot Angle", shootAngle);
        
        drivebase.getSwerveDrive().field.getObject("Target Point").setPose(new Pose2d(targetPos.getX(), targetPos.getY(), new Rotation2d()));
    }

    public static void updateBallPositionsAdvantageScope() {
        Pose3d[] positions = balls.stream().map(Ball::getPosition).toArray(Pose3d[]::new);
        
        ballPublisher.set(positions);
        SmartDashboard.putNumber("Simulation/Balls In Storage", storage.size());
    }

    public static Pose2d[] getBallPoses2d() {
        List<Pose2d> poses = new ArrayList<>();

        for (Ball ball : balls) {
            if (ball != null) {
                Pose3d pos = ball.getPosition();
                poses.add(new Pose2d(pos.getX(), pos.getY(), new Rotation2d()));
            }
        }
        
        return poses.toArray(new Pose2d[0]);
    }
}
