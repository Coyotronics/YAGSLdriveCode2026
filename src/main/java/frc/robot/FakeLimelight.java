package frc.robot;

public class FakeLimelight {
    private static double tx = 0.0;
    private static boolean hasTarget = true;

    public static double getTX() { return tx; }
    public static boolean hasValidTarget() { return hasTarget; }

    public static void setTX(double tx_) { tx = tx_; }
    public static void setHasTarget(boolean hasTarget_) { hasTarget = hasTarget_; }
}
