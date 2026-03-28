package frc.robot.commands;

import java.util.function.DoubleSupplier;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IntakeArm;
import frc.robot.subsystems.RollerSubsystem;
import frc.robot.subsystems.ShooterFlywheel;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utils.AllianceFlipUtil;
import frc.robot.constants.FieldConstants;

import edu.wpi.first.math.MathUtil;

import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.ShooterFlywheel;

public class AimHood extends Command {
  // --- Tunable field / robot constants ---
  private static final double G = 9.80665; // m/s^2

  // Hub center height from carpet. Replace with your game-specific number.
  private static final double HUB_HEIGHT_METERS = 2.64;

  // Height of the ball leaving your shooter/hood.
  private static final double SHOOTER_EXIT_HEIGHT_METERS = 0.90;

  // Radius of your shooter wheel in meters.
  // Example: 4 inch wheel => 0.1016 m diameter => 0.0508 m radius
  private static final double SHOOTER_WHEEL_RADIUS_METERS = 0.0508;

  // Empirical factor that converts wheel surface speed into real ball exit speed.
  // You will almost certainly need to tune this on the robot.
  private static final double EXIT_EFFICIENCY = 0.92;

  // Hood limits. Replace with your actual mechanical limits.
  private static final double MIN_HOOD_DEG = 10.0;
  private static final double MAX_HOOD_DEG = 60.0;

  private final HoodSubsystem hood;
  private final ShooterFlywheel flywheel;
  private final DoubleSupplier distanceMetersSupplier;

  public AimHood(
      HoodSubsystem hood,
      ShooterFlywheel flywheel,
      DoubleSupplier distanceMetersSupplier) {
    this.hood = hood;
    this.flywheel = flywheel;
    this.distanceMetersSupplier = distanceMetersSupplier;

    addRequirements(hood);
  }

  @Override
  public void execute() {
    double distanceMeters = distanceMetersSupplier.getAsDouble();

    // Use measured RPM from the subsystem, not just the setpoint.
    double flywheelRpm = flywheel.getTopRPM().magnitude();

    double ballExitSpeedMps = estimateBallExitSpeedMps(flywheelRpm);

    double deltaHeightMeters = HUB_HEIGHT_METERS - SHOOTER_EXIT_HEIGHT_METERS;

    double hoodAngleRad = solveLaunchAngleRadians(
        distanceMeters,
        deltaHeightMeters,
        ballExitSpeedMps);

    if (Double.isNaN(hoodAngleRad)) {
      return;
    }

    double hoodAngleDeg = Math.toDegrees(hoodAngleRad);
    hoodAngleDeg = MathUtil.clamp(hoodAngleDeg, MIN_HOOD_DEG, MAX_HOOD_DEG);

    hood.setAngleSetpoint(Degrees.of(hoodAngleDeg));
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  private double estimateBallExitSpeedMps(double flywheelRpm) {
    // Convert RPM -> wheel surface speed.
    double wheelSurfaceSpeedMps =
        (flywheelRpm / 60.0) * (2.0 * Math.PI * SHOOTER_WHEEL_RADIUS_METERS);

    // Convert to ball exit speed using an empirical efficiency factor.
    return wheelSurfaceSpeedMps * EXIT_EFFICIENCY;
  }

  /**
   * Solves for the lower-angle launch angle that hits the target.
   *
   * Equation:
   *   deltaY = d * tan(theta) - (g d^2) / (2 v^2 cos^2(theta))
   *
   * Rearranged into a quadratic in tan(theta).
   */
  private double solveLaunchAngleRadians(double d, double deltaY, double v) {
    if (d <= 0.0 || v <= 0.0) {
      return Double.NaN;
    }

    double v2 = v * v;
    double v4 = v2 * v2;

    // Discriminant:
    // v^4 - g(g d^2 + 2 deltaY v^2)
    double discriminant = v4 - G * (G * d * d + 2.0 * deltaY * v2);

    if (discriminant < 0.0) {
      // No real solution at this speed.
      return Double.NaN;
    }

    double sqrt = Math.sqrt(discriminant);

    // Lower-angle solution:
    // tan(theta) = (v^2 - sqrt(...)) / (g d)
    double tanTheta = (v2 - sqrt) / (G * d);

    double theta = Math.atan(tanTheta);

    // Optional sanity clamp in radians before converting back to degrees.
    double minRad = Math.toRadians(MIN_HOOD_DEG);
    double maxRad = Math.toRadians(MAX_HOOD_DEG);

    if (theta < minRad || theta > maxRad) {
      // Try the higher-angle branch if the lower one is outside hood limits.
      double tanThetaHigh = (v2 + sqrt) / (G * d);
      double thetaHigh = Math.atan(tanThetaHigh);

      if (thetaHigh >= minRad && thetaHigh <= maxRad) {
        return thetaHigh;
      }
    }

    return theta;
  }
}