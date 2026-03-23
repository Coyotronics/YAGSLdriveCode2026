package frc.robot.simulation;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Filesystem;

import java.io.File;
import java.util.*;

// srry, not explanation this time, not enough time to do that

public class FieldCollisionLoader {

    public static double length = 17.37;
    public static double width  = 7.99;

    public static double deltaX = (length / 2.0) - 0.4325;
    public static double deltaY = (width / 2.0) + 0.075;
    public static double deltaZ = 0;

    public static final Translation3d FIELD_OFFSET = new Translation3d(deltaX, deltaY, deltaZ);

    private static final double CELL_SIZE = 0.5;

    public static class SpatialGrid {
        public Map<Long, List<AABB>> cells = new HashMap<>();
    }

    public static class AABB {
        public Translation3d min, max;
        public AABB(Translation3d min, Translation3d max) { this.min = min; this.max = max; }
    }

    public static SpatialGrid loadFieldCollision() {
        File deployDir = Filesystem.getDeployDirectory();
        File assetFile = new File(deployDir, "assets/field_model.stl");

        List<Translation3d[]> triangles;

        try {
            triangles = parseSTL(assetFile);
        } catch (Exception e) {
            return null;
        }

        SpatialGrid grid = new SpatialGrid();

        for (Translation3d[] tri : triangles) {
            Translation3d min = computeMin(tri);
            Translation3d max = computeMax(tri);
            AABB box = new AABB(min, max);

            int minX = cell(min.getX());
            int minY = cell(min.getY());
            int minZ = cell(min.getZ());
            int maxX = cell(max.getX());
            int maxY = cell(max.getY());
            int maxZ = cell(max.getZ());

            for (int x = minX; x <= maxX; x++)
            for (int y = minY; y <= maxY; y++)
            for (int z = minZ; z <= maxZ; z++) {
                long key = hash(x,y,z);
                grid.cells.computeIfAbsent(key, k -> new ArrayList<>()).add(box);
            }
        }

        System.out.println("[Grid] Cells: " + grid.cells.size());
        return grid;
    }

    public static Pose3d[] getFieldVerticesAsPose3d(File stlFile) {
        try {
            List<Translation3d[]> triangles = parseSTL(stlFile);
            List<Pose3d> poses = new ArrayList<>();

            for (Translation3d[] tri : triangles) {
                for (Translation3d v : tri) {
                    poses.add(new Pose3d(
                        new Translation3d(
                            v.getX() + FIELD_OFFSET.getX(),
                            v.getY() + FIELD_OFFSET.getY(),
                            v.getZ() + FIELD_OFFSET.getZ()
                        ),
                        new Rotation3d()
                    ));
                }
            }

            return poses.toArray(new Pose3d[0]);
        } catch (Exception e) {
            return new Pose3d[0];
        }
    }


    public static Translation3d getFieldPushVector(SpatialGrid grid, Translation3d center, double radius) {
        int cx = cell(center.getX());
        int cy = cell(center.getY());
        int cz = cell(center.getZ());

        List<AABB> candidates = new ArrayList<>();

        for (int x = cx-1; x <= cx+1; x++)
        for (int y = cy-1; y <= cy+1; y++)
        for (int z = cz-1; z <= cz+1; z++) {
            List<AABB> cell = grid.cells.get(hash(x,y,z));
            if (cell != null) candidates.addAll(cell);
        }

        Translation3d bestPush = null;
        double maxPenetration = 0;

        for (AABB box : candidates) {
            Translation3d push = sphereAABBOverlap(center, radius, box.min, box.max);
            if (push != null) {
                double pen = push.getNorm();
                if (pen > maxPenetration) {
                    maxPenetration = pen;
                    bestPush = push;
                }
            }
        }

        return bestPush;
    }

    private static Translation3d sphereAABBOverlap(Translation3d sphere, double radius, Translation3d min, Translation3d max) {
        double cx = Math.max(min.getX(), Math.min(max.getX(), sphere.getX()));
        double cy = Math.max(min.getY(), Math.min(max.getY(), sphere.getY()));
        double cz = Math.max(min.getZ(), Math.min(max.getZ(), sphere.getZ()));

        double dx = sphere.getX() - cx;
        double dy = sphere.getY() - cy;
        double dz = sphere.getZ() - cz;

        double dist2 = dx*dx + dy*dy + dz*dz;
        if (dist2 >= radius*radius || dist2 < 1e-9) return null;

        double dist = Math.sqrt(dist2);
        double pen = radius - dist;
        return new Translation3d(dx/dist*pen, dy/dist*pen, dz/dist*pen);
    }

    private static int cell(double v) { return (int)Math.floor(v / CELL_SIZE); }

    // https://en.wikipedia.org/wiki/Fowler%E2%80%93Noll%E2%80%93Vo_hash_function
    
    private static long hash(int x, int y, int z) {
        long h = 1469598103934665603L;
        h ^= x; h *= 1099511628211L;
        h ^= y; h *= 1099511628211L;
        h ^= z; h *= 1099511628211L;
        return h;
    }

    private static Translation3d computeMin(Translation3d[] tri) {
        double x = Math.min(tri[0].getX(), Math.min(tri[1].getX(), tri[2].getX()));
        double y = Math.min(tri[0].getY(), Math.min(tri[1].getY(), tri[2].getY()));
        double z = Math.min(tri[0].getZ(), Math.min(tri[1].getZ(), tri[2].getZ()));
        return new Translation3d(x,y,z);
    }

    private static Translation3d computeMax(Translation3d[] tri) {
        double x = Math.max(tri[0].getX(), Math.max(tri[1].getX(), tri[2].getX()));
        double y = Math.max(tri[0].getY(), Math.max(tri[1].getY(), tri[2].getY()));
        double z = Math.max(tri[0].getZ(), Math.max(tri[1].getZ(), tri[2].getZ()));
        return new Translation3d(x,y,z);
    }

    private static List<Translation3d[]> parseSTL(File file) throws Exception {
        return STLParser.parse(file);
    }
}
