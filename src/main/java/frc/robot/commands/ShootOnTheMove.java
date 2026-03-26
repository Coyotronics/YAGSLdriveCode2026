package frc.robot.commands;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;

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

import frc.robot.constants.GenericConstants;

import swervelib.SwerveInputStream;
import static edu.wpi.first.units.Units.*;


public class ShootOnTheMove extends Command {
  private final SwerveSubsystem drivetrain;
  private final HoodSubsystem hood;
  private final ShooterFlywheel flywheel;
  private final IntakeArm intakeArm;
  private final RollerSubsystem roller;

  double loopPeriodSecs = 0.01; 

  private final SwerveInputStream originalStream;

  private final LinearFilter turretAngleFilter =
      LinearFilter.movingAverage((int) (0.1 / loopPeriodSecs));
  private final LinearFilter hoodAngleFilter =
      LinearFilter.movingAverage((int) (0.1 / loopPeriodSecs));

  private Rotation2d lastTurretAngle;
  private double lastHoodAngle;
  private Rotation2d turretAngle;
  private double hoodAngle = Double.NaN;
  private double turretVelocity;
  private double hoodVelocity;
  private AngularVelocity lastShootSpeed;

  public record LaunchingParameters(
      boolean isValid,
      Rotation2d turretAngle,
      double turretVelocity,
      double hoodAngle,
      double hoodVelocity,
      double flywheelSpeed) {}

  // Cache parameters
  private LaunchingParameters latestParameters = null;

  private static double minDistance;
  private static double maxDistance;
  private static double phaseDelay;
  private static final InterpolatingTreeMap<Double, Rotation2d> launchHoodAngleMap =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);
  private static final InterpolatingDoubleTreeMap launchFlywheelSpeedMap =
      new InterpolatingDoubleTreeMap();

  private static final InterpolatingDoubleTreeMap timeOfFlightMap =
      new InterpolatingDoubleTreeMap();

  
  //ALL OF THE BELOW VALUES NEED TO BE ACCURATELY MEASURED AND TESTED, THEY ARE PLACEHOLDERS
  static {
    minDistance = 1.34;
    maxDistance = 5.60;
    phaseDelay = 0.03; // should be .13?

    launchHoodAngleMap.put(1.34, Rotation2d.fromDegrees(19.0)); // distance to target, hood angle
    launchHoodAngleMap.put(1.78, Rotation2d.fromDegrees(19.0)); 
    launchHoodAngleMap.put(2.17, Rotation2d.fromDegrees(24.0));
    launchHoodAngleMap.put(2.81, Rotation2d.fromDegrees(27.0));
    launchHoodAngleMap.put(3.82, Rotation2d.fromDegrees(29.0));
    launchHoodAngleMap.put(4.09, Rotation2d.fromDegrees(30.0));
    launchHoodAngleMap.put(4.40, Rotation2d.fromDegrees(31.0));
    launchHoodAngleMap.put(4.77, Rotation2d.fromDegrees(32.0));
    launchHoodAngleMap.put(5.57, Rotation2d.fromDegrees(32.0));
    launchHoodAngleMap.put(5.60, Rotation2d.fromDegrees(35.0));

    launchFlywheelSpeedMap.put(2.12923 - .22, 2725.0);  // distance, exit velocity
    launchFlywheelSpeedMap.put(2.504222 - .22, 2800.0);
    launchFlywheelSpeedMap.put(2.889 - .22, 2950.0);
    launchFlywheelSpeedMap.put(3.254686 - .22, 3085.0);
    launchFlywheelSpeedMap.put(3.695324 - .22, 3200.0);
    launchFlywheelSpeedMap.put(3.983757 - .22, 3320.0);
    launchFlywheelSpeedMap.put(4.498437 - .22, 3475.0);
    launchFlywheelSpeedMap.put(4.986071 - .22, 3675.0);
    launchFlywheelSpeedMap.put(5.410986 - .22, 4000.0);

    timeOfFlightMap.put(5.68, 1.16);
    timeOfFlightMap.put(4.55, 1.12);
    timeOfFlightMap.put(3.15, 1.11);
    timeOfFlightMap.put(1.88, 1.09);
    timeOfFlightMap.put(1.38, 0.90);
  }

  public ShootOnTheMove(
      SwerveSubsystem drivetrain, HoodSubsystem hood, ShooterFlywheel flywheel, IntakeArm intakeArm, RollerSubsystem roller, SwerveInputStream originalStream) {
    this.drivetrain = drivetrain;
    this.hood = hood;
    this.flywheel = flywheel;
    this.intakeArm = intakeArm;
    this.roller = roller;
    this.originalStream = originalStream;
  }

  @Override
  public void initialize() {
    super.initialize();

    lastHoodAngle = hood.getAngle().in(Units.Degrees);
    lastTurretAngle = drivetrain.getHeading();
    lastShootSpeed = flywheel.getSpeed();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void execute() {

    // Calculate estimated pose while accounting for phase delay
    Pose2d estimatedPose = drivetrain.getPose();
    ChassisSpeeds robotRelativeVelocity = drivetrain.getRobotVelocity();
    estimatedPose =
        estimatedPose.exp(
            new Twist2d(
                robotRelativeVelocity.vxMetersPerSecond * phaseDelay,
                robotRelativeVelocity.vyMetersPerSecond * phaseDelay,
                robotRelativeVelocity.omegaRadiansPerSecond * phaseDelay));

    // Calculate distance from turret to target
    Pose2d turretPosition =
        drivetrain.getPose();

    // Designate desired target

    Translation2d target =
        AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d());

    // if (ScoringSystem.CustomTriggers.scoringZone.getTrigger().getAsBoolean()) {
    //   target = AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d());
    // }

    // if (ScoringSystem.CustomTriggers.bumpZone.getTrigger().getAsBoolean()) {
    //   target = AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d());
    // }

    // if (ScoringSystem.CustomTriggers.leftNeutralZone.getTrigger().getAsBoolean()) {
    //   target =
    //       AllianceFlipUtil.apply(
    //           FieldConstants.LeftBump.nearRightCorner.plus(
    //               new Translation2d(0, Inches.of(36.5).in(Meters))));
    // }

    // if (ScoringSystem.CustomTriggers.rightNeutralZone.getTrigger().getAsBoolean()) {
    //   target =
    //       AllianceFlipUtil.apply(
    //           FieldConstants.RightBump.nearLeftCorner.plus(
    //               new Translation2d(0, -Inches.of(36.5).in(Meters))));
    // } 
    //NEED TO FIX ABOVE with custom triggers class in utils

    double turretToTargetDistance = target.getDistance(turretPosition.getTranslation());

    // Calculate field relative turret velocity
    ChassisSpeeds robotVelocity = drivetrain.getFieldVelocity();
    double robotAngle = estimatedPose.getRotation().getRadians();
    double turretVelocityX =
        robotVelocity.vxMetersPerSecond;;
    double turretVelocityY =
        robotVelocity.vyMetersPerSecond;
    //as you see, pro gamer moves

    // recursive tof
    // Account for imparted velocity by robot (turret) to offset
    double timeOfFlight;
    Pose2d lookaheadPose = turretPosition;
    double lookaheadTurretToTargetDistance = turretToTargetDistance;
    Pose2d newTarget = new Pose2d(target, Rotation2d.fromDegrees(0));
    for (int i = 0; i < 20; i++) {
      timeOfFlight = timeOfFlightMap.get(lookaheadTurretToTargetDistance);
      double offsetX = turretVelocityX * timeOfFlight;
      double offsetY = turretVelocityY * timeOfFlight;
      lookaheadPose =
          new Pose2d(
              turretPosition.getTranslation().plus(new Translation2d(offsetX, offsetY)),
              turretPosition.getRotation());
    
       newTarget = new Pose2d(target.plus(new Translation2d(offsetX, offsetY)), Rotation2d.fromDegrees(0));

      lookaheadTurretToTargetDistance = target.getDistance(lookaheadPose.getTranslation());
    }


    // I have no idea if this actually works, but it should in theory account for the robot's movement during the ball's flight and adjust the turret accordingly
    turretAngle =
        target.minus(lookaheadPose.getTranslation()).getAngle().minus(estimatedPose.getRotation());
    hoodAngle = launchHoodAngleMap.get(lookaheadTurretToTargetDistance).getRadians();
    if (lastTurretAngle == null) lastTurretAngle = turretAngle;
    if (Double.isNaN(lastHoodAngle)) lastHoodAngle = hoodAngle;
    turretVelocity =
        turretAngleFilter.calculate(
            turretAngle.minus(lastTurretAngle).getRadians()
                / loopPeriodSecs);
    hoodVelocity =
        hoodAngleFilter.calculate(
            (hoodAngle - lastHoodAngle) / loopPeriodSecs);
    lastTurretAngle = turretAngle;
    lastHoodAngle = hoodAngle;
    latestParameters =
        new LaunchingParameters(
            lookaheadTurretToTargetDistance >= minDistance
                && lookaheadTurretToTargetDistance <= maxDistance,
            turretAngle,
            turretVelocity,
            hoodAngle,
            hoodVelocity,
            launchFlywheelSpeedMap.get(lookaheadTurretToTargetDistance));

    lastShootSpeed = RPM.of(launchFlywheelSpeedMap.get(lookaheadTurretToTargetDistance));
    // SmartDashboard.putNumber("distance to turret", lookaheadTurretToTargetDistance);

    flywheel.setBothVelocitySetpoint(RPM.of(launchFlywheelSpeedMap.get(lookaheadTurretToTargetDistance)));
    roller.setVelocitySetpoint(RPM.of(launchFlywheelSpeedMap.get(lookaheadTurretToTargetDistance)));

    hood.setAngleSetpoint(Rotations.of(hoodAngle));

    
    SwerveInputStream modifiedStream = originalStream.copy().aim(newTarget);
    drivetrain.driveFieldOriented(modifiedStream);

    SmartDashboard.putNumber("Distance to Target", lookaheadTurretToTargetDistance);
    // SmartDashboard.putData("turretPosition", turretPosition);
  }

  public Trigger inScoringZone(Pose2d turretPose) {
    return new Trigger(
        () ->
            new Rectangle2d(
                    AllianceFlipUtil.apply(new Translation2d(0, 0)),
                    AllianceFlipUtil.apply(
                        new Translation2d(
                            FieldConstants.LinesVertical.starting, FieldConstants.fieldWidth)))
                .contains(turretPose.getTranslation()));
  }

  public Trigger inRightNeutralZone(Pose2d turretPose) {
    return new Trigger(
        () ->
            new Rectangle2d(
                    AllianceFlipUtil.apply(
                        new Translation2d(FieldConstants.LinesVertical.starting, 0)),
                    AllianceFlipUtil.apply(
                        new Translation2d(
                            FieldConstants.LinesVertical.oppAllianceZone,
                            FieldConstants.LinesHorizontal.center)))
                .contains(turretPose.getTranslation()));
  }

  public Trigger inLeftNeutralZone(Pose2d turretPose) {
    return new Trigger(
        () ->
            new Rectangle2d(
                    AllianceFlipUtil.apply(
                        new Translation2d(
                            FieldConstants.LinesVertical.starting,
                            FieldConstants.LinesHorizontal.center)),
                    AllianceFlipUtil.apply(
                        new Translation2d(
                            FieldConstants.LinesVertical.oppAllianceZone,
                            FieldConstants.fieldWidth)))
                .contains(turretPose.getTranslation()));
  }

  @Override
  public void end(boolean interupted) { 
    hood.stopCommand().schedule();
    flywheel.stopCommand().schedule();  
    roller.stopCommand().schedule();
  }
}