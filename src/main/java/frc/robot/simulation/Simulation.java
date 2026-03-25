package frc.robot.simulation;

import java.io.File;
import java.util.ArrayList;
import java.util.Arrays;
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
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

// commented it a lot if ur curious
// take that back, too lazy to comment more

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

    private static StructArrayPublisher<Pose3d> fieldVertexPublisher;

    private static List<Translation3d[]> fieldTriangles = new ArrayList<>();

    private static StructArrayPublisher<Pose3d> fieldTrianglePublisher;

    private static List<Ball> balls = new ArrayList<>();
    private static List<Ball> storage = new ArrayList<>();

    private static double shootAngle = 0.0;
    private static double shootSpeed = 0.0;

    public static final Translation3d shooterPosition = new Translation3d(0.0, 0.0, 0.0);

    private static FieldCollisionLoader.SpatialGrid fieldGrid;

    private static final double BALL_CELL = 0.4;
    
    private static final int GW = 50;
    private static final int GH = 24;
    private static final int GD = 10;

    private static final int MAX_CELL  = 16;
    private static final int[][] bGrid = new int[GW * GH * GD][MAX_CELL];
    private static final int[] bCnt = new int[GW * GH * GD];

    private static Pose3d[] ballPoseBuf = new Pose3d[0];
    private static Pose3d[] vertexBuf = new Pose3d[0];
    private static Pose3d[] intakeBuf = new Pose3d[0];

    private static int tick = 0;
    private static final int PUB_EVERY= 3;

    private static int gcX(double v) {
        return Math.max(0, Math.min(GW - 1, (int) Math.floor(v / BALL_CELL)));
    }
    
    private static int gcY(double v) {
        return Math.max(0, Math.min(GH - 1, (int) Math.floor(v / BALL_CELL)));
    }

    private static int gcZ(double v) {
        return Math.max(0, Math.min(GD - 1, (int) Math.floor(v / BALL_CELL)));
    }

    private static int gi(int x, int y, int z) {
        return x + y * GW + z * GW * GH;
    }

    public static void makeBalls(double centerX, double centerY, int cols, int rows, double spacing) {
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
    }

    public static void init() {
        // making balls in the center

        balls.clear();
        storage.clear();

        double centerX = (length / 2.0) - 0.4325;
        double centerY = (width / 2.0) + 0.075;

        int cols = 12;
        int rows = 30;
        double spacing = 0.2;

        makeBalls(centerX, centerY, cols, rows, spacing);

        // later add the balls on each teams sides
        //makeBalls(0.5, centerY + 2, 6, 4, spacing);

        // so u can see stuff in advantage scope

        ballPublisher = NetworkTableInstance.getDefault().getStructArrayTopic("/Field/Ball Positions", Pose3d.struct).publish();
        vertexPublisher = NetworkTableInstance.getDefault().getStructArrayTopic("/Field/Robot Vertices", Pose3d.struct).publish();
        intakePublisher = NetworkTableInstance.getDefault().getStructArrayTopic("/Field/Robot Intake", Pose3d.struct).publish();

        fieldVertexPublisher = NetworkTableInstance.getDefault().getStructArrayTopic("/Field/Field Vertices", Pose3d.struct).publish();
    
        File stlVertFile = new File(Filesystem.getDeployDirectory(), "assets/field_model.stl");
        Pose3d[] verts = FieldCollisionLoader.getFieldVerticesAsPose3d(stlVertFile);
        int sampleCount = Math.min(500, verts.length);
        Pose3d[] sampleVert = new Pose3d[sampleCount];
        
        for (int i = 0; i < sampleCount; i++)
            sampleVert[i] = verts[(int) ((long) i * verts.length / sampleCount)];
        
        System.out.println("Publishing " + sampleVert.length + " Sampled Vertices of " + verts.length + " Total");
        fieldVertexPublisher.set(sampleVert);
        
        fieldTrianglePublisher = NetworkTableInstance.getDefault().getStructArrayTopic("/Field/Field Triangles", Pose3d.struct).publish();

        File stlFile = new File(Filesystem.getDeployDirectory(), "assets/field_model.stl");

        try {
            List<Translation3d[]> triangles = STLParser.parse(stlFile);
            fieldTriangles.addAll(triangles);
            System.out.println("Loaded Triangles: " + fieldTriangles.size());
        } catch (Exception e) {
            System.out.println("ERROR: " + e.getMessage());
        }

        int publishCount = Math.min(500, fieldTriangles.size());
        Pose3d[] sample = new Pose3d[publishCount * 3];

        for (int i = 0; i < publishCount; i++) {
            int index = (int)((long)i * fieldTriangles.size() / publishCount);
            Translation3d[] tri = fieldTriangles.get(index);
            sample[i * 3 + 0] = new Pose3d(
                new Translation3d(
                    tri[0].getX() + FieldCollisionLoader.FIELD_OFFSET.getX(),
                    tri[0].getY() + FieldCollisionLoader.FIELD_OFFSET.getY(),
                    tri[0].getZ() + FieldCollisionLoader.FIELD_OFFSET.getZ()
                ),
                new Rotation3d()
            );

            sample[i * 3 + 1] = new Pose3d(
                new Translation3d(
                    tri[1].getX() + FieldCollisionLoader.FIELD_OFFSET.getX(),
                    tri[1].getY() + FieldCollisionLoader.FIELD_OFFSET.getY(),
                    tri[1].getZ() + FieldCollisionLoader.FIELD_OFFSET.getZ()
                ),
                new Rotation3d()
            );

            sample[i * 3 + 2] = new Pose3d(
                new Translation3d(
                    tri[2].getX() + FieldCollisionLoader.FIELD_OFFSET.getX(),
                    tri[2].getY() + FieldCollisionLoader.FIELD_OFFSET.getY(),
                    tri[2].getZ() + FieldCollisionLoader.FIELD_OFFSET.getZ()
                ),
                new Rotation3d()
            );
        }

        fieldTrianglePublisher.set(sample);

        fieldGrid = FieldCollisionLoader.loadFieldCollision();
    }
    
    public static void updateVertexPositionsAdvantageScope(Translation3d[] points, Translation3d[] intakePoints, Pose3d origin) {
        if (vertexBuf.length != points.length) vertexBuf  = new Pose3d[points.length];
        if (intakeBuf.length != intakePoints.length) intakeBuf  = new Pose3d[intakePoints.length];
        
        for (int i = 0; i < points.length; i++) vertexBuf[i] = transformToWorld(points[i],       origin);
        for (int i = 0; i < intakePoints.length; i++) intakeBuf[i]  = transformToWorld(intakePoints[i], origin);
        
        if (tick % PUB_EVERY == 0) {
            vertexPublisher.set(vertexBuf);
            intakePublisher.set(intakeBuf);
        }
    }

    public static void shootBall(Pose3d robotPose) {
        if (storage.isEmpty()) return;

        Ball ball = storage.remove(0);
        
        Pose3d spawnPose = transformToWorld(shooterPosition, robotPose);

        // yaw is the heading (rotation on the z axis)
        double yaw = robotPose.getRotation().getZ() + (Math.PI / 2.0);
        double elevationRadians = Math.toRadians(shootAngle);

        // horizotnal velocity is the speed along the ground (xy plane) and vy is the speed vertically (z axis)
        double horizotnalVelocity = shootSpeed * Math.cos(elevationRadians);
        double vz  = shootSpeed * Math.sin(elevationRadians);

        // splits speed along the xy plane into speeds correlating to their axis
        double vx = horizotnalVelocity * Math.cos(yaw);
        double vy = horizotnalVelocity * Math.sin(yaw);

        // put the ball at the shooter and let if fly :)
        ball.setPosition(spawnPose);
        ball.setVelocity(new Translation3d(vx, vy, vz));
        ball.setIgnoreRobot(true);

        // finally add the ball background storage to the "world"
        balls.add(ball);
    }

    public static void updateBalls(Translation3d[] points, Translation3d[] intakePoints, Pose3d origin) {
        tick++;
        
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

                    ball.setPosition(ball.x + push.getX(), ball.y + push.getY(), ball.z + push.getZ());

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

            if (ball.x >= 11.40 && ball.x <= 12.40 && ball.y >= 3.55 && ball.y <= 4.55 && ball.z >= 0 && ball.z <= 1.5) {
                ball.setPosition(10.9, 4.05, 1);
                ball.setVelocity(-0.03, (Math.random() - 0.5) * 0.05, -0.03);
            }

            if (fieldGrid != null) {
                Translation3d ballWorld = new Translation3d(
                    ball.x - FieldCollisionLoader.FIELD_OFFSET.getX(),
                    ball.y - FieldCollisionLoader.FIELD_OFFSET.getY(),
                    ball.z - FieldCollisionLoader.FIELD_OFFSET.getZ()
                );
                Translation3d fieldPush = FieldCollisionLoader.getFieldPushVector(fieldGrid, ballWorld, ball.getRadius());
                if (fieldPush != null) {
                    ball.setPosition(ball.x + fieldPush.getX(), ball.y + fieldPush.getY(), ball.z + fieldPush.getZ());
                    double mag = fieldPush.getNorm();
                    if (mag > 1e-9) {
                        double impulse = Math.min(mag, 0.02);
                        ball.addVelocity(fieldPush.getX()/mag * impulse, fieldPush.getY()/mag * impulse, fieldPush.getZ()/mag * impulse);
                    }
                }
            }
        }

        // collisions between the balls
        handleBallCollisions();
    }

    private static void handleBallCollisions() {
        int ballCount = balls.size();
        if (ballCount < 2) return;

        Arrays.fill(bCnt, 0);

        for (int i = 0; i < ballCount; i++) {
            Pose3d pos = balls.get(i).getPosition();
            int gridIndex  = gi(gcX(pos.getX()), gcY(pos.getY()), gcZ(pos.getZ()));
            if (bCnt[gridIndex] < MAX_CELL) bGrid[gridIndex][bCnt[gridIndex]++] = i;
        }

        for (int i = 0; i < ballCount; i++) {
            Ball a = balls.get(i);

            // cells
            int ax = gcX(a.x);
            int ay = gcY(a.y);
            int az = gcZ(a.z);

            // the same balls in the same grid, check it
            for (int dx = -1; dx <= 1; dx++) {
                int nx = ax + dx; if (nx < 0 || nx >= GW) continue;
                
                for (int dy = -1; dy <= 1; dy++) {
                    int ny = ay + dy; if (ny < 0 || ny >= GH) continue;
                    
                    for (int dz = -1; dz <= 1; dz++) {
                        int nz = az + dz; if (nz < 0 || nz >= GD) continue;
                        int cellIdx = gi(nx, ny, nz);
                        int count   = bCnt[cellIdx];
                        for (int k = 0; k < count; k++) {
                            int j = bGrid[cellIdx][k];
                            if (j <= i) continue;
                            
                            Ball b = balls.get(j);

                            // already got dx so yea, dont let the extra d confuse u
                            double ddx = b.x - a.x;
                            double ddy = b.y - a.y;
                            double ddz = b.z - a.z;
                            
                            double dist2 = ddx*ddx + ddy*ddy + ddz*ddz;
                            double minD  = a.getRadius() + b.getRadius();
                            if (dist2 >= minD * minD || dist2 < 1e-18) continue;
                            double dist  = Math.sqrt(dist2);
                            
                            double overlap = (minD - dist) * 0.5;
                            double nx2 = ddx/dist, ny2 = ddy/dist, nz2 = ddz/dist;
                            
                            a.setPosition(a.x - nx2*overlap, a.y - ny2*overlap, a.z - nz2*overlap);
                            b.setPosition(b.x + nx2*overlap, b.y + ny2*overlap, b.z + nz2*overlap);
                            
                            double dot = (a.vx-b.vx)*nx2 + (a.vy-b.vy)*ny2 + (a.vz-b.vz)*nz2;
                            
                            if (dot > 0) {
                                // friggin impiulse
                                double impulse = dot * 0.8;
                                a.setVelocity(a.vx - impulse*nx2, a.vy - impulse*ny2, a.vz - impulse*nz2);
                                b.setVelocity(b.vx + impulse*nx2, b.vy + impulse*ny2, b.vz + impulse*nz2);
                            }
                        }
                    }
                }
            }
        }
    }

    private static boolean isBallInIntake(Ball ball, Translation3d[] intakePoints, Pose3d origin) {
        double yaw  = origin.getRotation().getZ();

        // save it for later
        double cosY = Math.cos(-yaw);
        double sinY = Math.sin(-yaw);
        
        // difference between the ball and the origin
        double dx = ball.x - origin.getX();
        double dy = ball.y - origin.getY();
        double dz = ball.z - origin.getZ();

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
        double newX = MathUtil.clamp(ball.x, ballRadius, length - ballRadius);
        double newY = MathUtil.clamp(ball.y, ballRadius, width  - ballRadius);
        
        if (newX != ball.x) { ball.vx *= -0.5; ball.x = newX; }
        if (newY != ball.y) { ball.vy *= -0.5; ball.y = newY; }
        
        if (ball.z < ballRadius) {
            ball.z = ballRadius;
            
            if (ball.vz < 0)
                ball.vz *= -0.4;
        }
    }

    private static Translation3d getPushVector(Ball ball, Translation3d[] points, Pose3d origin) {
        double yaw  = origin.getRotation().getZ();
        
        double cosY = Math.cos(-yaw);
        double sinY = Math.sin(-yaw);

        // difference
        double dx = ball.x - origin.getX();
        double dy = ball.y - origin.getY();
        double dz = ball.z - origin.getZ();

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
        // LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");

        Pose2d robotPose2d = drivebase.getPose();
 
        double dx = targetPos.getX() - robotPose2d.getX();
        double dy = targetPos.getY() - robotPose2d.getY();
 
        // ignore height difference
        double horizontalDistance = Math.sqrt(dx * dx + dy * dy);

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
        if (tick % PUB_EVERY != 0) return;

        int n = balls.size();
        if (ballPoseBuf.length != n) ballPoseBuf = new Pose3d[n];
        for (int i = 0; i < n; i++) ballPoseBuf[i] = balls.get(i).getPosition();
        
        ballPublisher.set(ballPoseBuf);
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
