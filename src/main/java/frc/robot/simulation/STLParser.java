package frc.robot.simulation;

import edu.wpi.first.math.geometry.Translation3d;

import java.io.File;
import java.io.FileInputStream;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.ArrayList;
import java.util.List;

// srry, not explanation this time, not enough time to do that

public class STLParser {
    public static List<Translation3d[]> parse(File file) throws Exception {
        List<Translation3d[]> triangles = new ArrayList<>();

        // it filters tiny details
        final double MIN_SIDE = 0.02;
        final double MAX_SIDE = 3.0;

        try (FileInputStream fis = new FileInputStream(file)) {
            fis.skip(80);

            byte[] countBuffer = new byte[4];
            fis.read(countBuffer);
            int numTriangles = ByteBuffer.wrap(countBuffer).order(ByteOrder.LITTLE_ENDIAN).getInt();

            for (int i = 0; i < numTriangles; i++) {
                byte[] facetBuffer = new byte[50];
                fis.read(facetBuffer);
                ByteBuffer bb = ByteBuffer.wrap(facetBuffer).order(ByteOrder.LITTLE_ENDIAN);
                bb.position(12);

                Translation3d[] vertices = new Translation3d[3];
                for (int v = 0; v < 3; v++) {
                    float x = bb.getFloat();
                    float y = bb.getFloat();
                    float z = bb.getFloat();
                    vertices[v] = new Translation3d(x, y, z);
                }

                boolean good = true;

                for (int a = 0; a < 3; a++) {
                    int b = (a + 1) % 3;
                    double dx = vertices[b].getX() - vertices[a].getX();
                    double dy = vertices[b].getY() - vertices[a].getY();
                    double dz = vertices[b].getZ() - vertices[a].getZ();

                    double sideLen = Math.sqrt(dx*dx + dy*dy + dz*dz);

                    if (sideLen < MIN_SIDE || sideLen > MAX_SIDE) {
                        good = false;
                        break;
                    }
                }

                if (good) triangles.add(vertices);
            }
        }

        return triangles;
    }
}
