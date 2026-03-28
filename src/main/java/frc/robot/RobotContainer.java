// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.Constants.OperatorConstants;
// import frc.robot.commands.AlignSwerveCommand;
// import frc.robot.simulation.Simulation;
import frc.robot.commands.AimHood;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.OperatorConstants;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IntakeArm;
import frc.robot.subsystems.IntakeRollers;
import frc.robot.subsystems.RollerSubsystem;
import frc.robot.subsystems.ShooterFlywheel;
import frc.robot.subsystems.SwerveSubsystem;

import static edu.wpi.first.units.Units.*;


import swervelib.SwerveInputStream;


import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;

import java.io.File;
import java.io.IOException;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicReference;

import org.json.simple.parser.ParseException;
//import org.photonvision.targeting.PhotonPipelineResult;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.SwerveDriveTest;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveControllerConfiguration;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import frc.robot.LimelightHelpers;
import frc.robot.constants.*;
import limelight.Limelight;
import edu.wpi.first.math.VecBuilder;

import limelight.Limelight;
import limelight.networktables.AngularVelocity3d;
import limelight.networktables.LimelightPoseEstimator;
import limelight.networktables.LimelightResults;
import limelight.networktables.LimelightSettings.LEDMode;
import limelight.networktables.Orientation3d;
import limelight.networktables.PoseEstimate;
import limelight.networktables.LimelightPoseEstimator.EstimationMode;
import limelight.networktables.target.pipeline.NeuralClassifier;

import static edu.wpi.first.units.Units.*;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import java.util.function.*; 

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

  // Replace with CommandPS4Controller or CommandJoystick if needed
  final         CommandXboxController driverXbox = new CommandXboxController(0);
  // The robot's subsystems and commands are defined here...
  public final SwerveSubsystem       drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                "swerve/neo"));
  public final RollerSubsystem      Conveyor      = new RollerSubsystem();
  public final ShooterFlywheel      ShooterFlywheel = new ShooterFlywheel();

  public final HoodSubsystem      HoodSubsystem = new HoodSubsystem();
  public final IntakeArm          IntakeArm = new IntakeArm();

  public final IntakeRollers      IntakeRollers = new IntakeRollers();


  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> driverXbox.getLeftY() * -1,
                                                                () -> driverXbox.getLeftX() * -1)
                                                            .withControllerRotationAxis(() -> driverXbox.getRawAxis(4)*-1)
                                                            .deadband(Constants.OperatorConstants.DEADBAND)
                                                            .scaleTranslation(0.9)
                                                            .allianceRelativeControl(true);


    SwerveInputStream test = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> 0,
                                                                () -> 0)
                                                            .withControllerRotationAxis(()->0)
                                                            .deadband(Constants.OperatorConstants.DEADBAND)
                                                            .scaleTranslation(0.9)
                                                            .allianceRelativeControl(true);
   
  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverXbox::getRightX,
                                                                                             driverXbox::getRightY)
                                                           .headingWhile(true);

  /**
   * Clone's the angular velocity input stream and converts it to a robotRelative input stream.
   */
  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
                                                             .allianceRelativeControl(false);

  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                        () -> -driverXbox.getLeftY(),
                                                                        () -> -driverXbox.getLeftX())
                                                                    .withControllerRotationAxis(driverXbox :: getRightX)
                                                                    .deadband(OperatorConstants.DEADBAND)
                                                                    .scaleTranslation(0.8)
                                                                    .allianceRelativeControl(true)
                                                                    .headingWhile(false);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleKeyboard     = driveAngularVelocityKeyboard.copy()
                                                                               .withControllerHeadingAxis(() ->
                                                                                                              Math.sin(
                                                                                                                  driverXbox.getRawAxis(
                                                                                                                      2) *
                                                                                                                    Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2),
                                                                                                          () ->
                                                                                                              Math.cos(
                                                                                                                  driverXbox.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2))
                                                                               .headingWhile(true)
                                                                               .translationHeadingOffset(true)
                                                                               .translationHeadingOffset(Rotation2d.fromDegrees(
                                                                                   0));

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);

    defaultCommands();

    NamedCommands.registerCommand("IntakeWithAuto", getAutonomousCommand());
        
  }

  public HoodSubsystem GetHoodSubsystem() {
    return HoodSubsystem;
  }
    
  private void defaultCommands() {

    Command testCommand = drivebase.driveFieldOriented(test);
    Command driveFieldOrientedDirectAngle      = drivebase.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveRobotOrientedAngularVelocity  = drivebase.driveFieldOriented(driveRobotOriented);
    
    Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngle);
    Command driveFieldOrientedDirectAngleKeyboard      = drivebase.driveFieldOriented(driveDirectAngleKeyboard);
    Command driveFieldOrientedAnglularVelocityKeyboard = drivebase.driveFieldOriented(driveAngularVelocityKeyboard);
    Command driveSetpointGenKeyboard = drivebase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngleKeyboard);

     if (RobotBase.isSimulation())
    {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    } else
    {
      System.out.println("Setting default command");
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
      //drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
      //drivebase.setDefaultCommand(driveFieldOrientedDirectAngle);
      System.out.println("I lowk did this shit");
    }


    // Conveyor.setDefaultCommand(
    //          Commands.run(() -> Conveyor.setDutyCycleSetpoint(
    //                             Math.max(-1, Math.min(1, -1*driverXbox.getRightTriggerAxis()))
    //                             ), Conveyor));

    // ShooterFlywheel.setDefaultCommand(
    //          Commands.run(() -> ShooterFlywheel.setBothDutyCycleSetpoint(
    //                             Math.max(-1, Math.min(1, driverXbox.getRightTriggerAxis()))
    //                             ), ShooterFlywheel));

    
    Conveyor.setDefaultCommand(Commands.run(() -> Conveyor.setDutyCycle(0), Conveyor));

    ShooterFlywheel.setDefaultCommand(Commands.run(() -> ShooterFlywheel.setBothDutyCycleSetpoint(0), ShooterFlywheel)); // TODO: make sure to take this away

    
   

    // HoodSubsystem.setDefaultCommand(Commands.run(() -> HoodSubsystem.setVelocity(0), HoodSubsystem));
    HoodSubsystem.setDefaultCommand(Commands.run(() -> {HoodSubsystem.setVelocity(0);}, HoodSubsystem));

    IntakeArm.setDefaultCommand(Commands.run(() -> IntakeArm.setDutyCycleSetpoint(0), IntakeArm));

    IntakeRollers.setDefaultCommand(Commands.run(() -> IntakeRollers.setDutyCycleSetpoint(0), IntakeRollers)); 
    //Change this to 0.5 or something when comp comes around, also use set velocity instead of duty cycle for PID control. this rn is for testing

    

    
  

  }
    
      /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {

// SwerveDrive swerveDrive = drivebase.getSwerveDrive();
//     // 1
// Supplier<Pose2d> targetPoseSupplier = () -> vision.getTagPose(tagId);

// // 2
// DoubleSupplier xSup = () -> -driver.getLeftY();
// DoubleSupplier ySup = () -> -driver.getLeftX();

// // 3
// DoubleSupplier headingXSup = () -> {
//     Pose2d robot = swerveDrive.getPose();
//     Pose2d target = targetPoseSupplier.get();
//     double dx = target.getX() - robot.getX();
//     double dy = target.getY() - robot.getY();
//     double angle = Math.atan2(dy, dx);
//     return Math.cos(angle);
// };

// DoubleSupplier headingYSup = () -> {
//     Pose2d robot = swerveDrive.getPose();
//     Pose2d target = targetPoseSupplier.get();
//     double dx = target.getX() - robot.getX();
//     double dy = target.getY() - robot.getY();
//     double angle = Math.atan2(dy, dx);
//     return Math.sin(angle);
// };

// 4
// SwerveInputStream driveStream = SwerveInputStream.of(swerveDrive, xSup, ySup)
    // .withControllerHeadingAxis(headingXSup, headingYSup)
    // .deadband(OperatorConstants.DEADBAND)
    // .scaleTranslation(0.8)
    // .headingWhile(() -> true);


   

    // if (Robot.isSimulation())
    // {
    //   Pose2d target = new Pose2d(new Translation2d(1, 4),
    //                              Rotation2d.fromDegrees(90));
    //   //drivebase.getSwerveDrive().field.getObject("targetPose").setPose(target);
    //   driveDirectAngleKeyboard.driveToPose(() -> target,
    //                                        new ProfiledPIDController(5,
    //                                                                  0,
    //                                                                  0,
    //                                                                  new Constraints(5, 2)),
    //                                        new ProfiledPIDController(5,
    //                                                                  0,
    //                                                                  0,
    //                                                                  new Constraints(Units.degreesToRadians(360),
    //                                                                                  Units.degreesToRadians(180))
    //                                        ));
    //   driverXbox.start().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
    //   driverXbox.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());
    //   driverXbox.button(2).whileTrue(Commands.runEnd(() -> driveDirectAngleKeyboard.driveToPoseEnabled(true),
    //                                                  () -> driveDirectAngleKeyboard.driveToPoseEnabled(false)));
  


    //   driverXbox.povUp().whileTrue(Commands.run(() -> HoodSubsystem.setVelocity(10), HoodSubsystem));
    //   driverXbox.povDown().whileTrue(Commands.run(() -> HoodSubsystem.setVelocity(-10), HoodSubsystem));

    // }
    // else
    // {

      if (Robot.isSimulation()) {
        driverXbox.a().onTrue(Commands.runOnce(() -> {
          Pose2d robotPose = drivebase.getSwerveDrive().getSimulationDriveTrainPose().orElse(drivebase.getPose());
         // Simulation.shootBall(new Pose3d(robotPose.getX(), robotPose.getY(), 0, new Rotation3d(robotPose.getRotation())));
        }));
      }

      // driverXbox.povUp().whileTrue(Commands.run(() -> 
      //                                         {HoodSubsystem.setVelocity(10);
      //                                           System.out.println(HoodSubsystem.getAngle().in(Degrees));}, HoodSubsystem));
      // driverXbox.povDown().whileTrue(Commands.run(() -> 
      //                                            {HoodSubsystem.setVelocity(-10);
      //                                              System.out.println(HoodSubsystem.getAngle().in(Degrees));}, HoodSubsystem));

      driverXbox.povUp().whileTrue(Commands.run(() -> 
                                              {
                                                HoodSubsystem.setVelocity(10);
                                                System.out.println("Hood set to 10");
                                                System.out.println("Hood angle: " + HoodSubsystem.getAngle());
                                                }, HoodSubsystem));
      driverXbox.povDown().whileTrue(Commands.run(() -> 
                                                 {HoodSubsystem.setVelocity(-10);
                                                   System.out.println("Hood set to -10");
                                                    System.out.println("Hood angle: " + HoodSubsystem.getAngle());}, HoodSubsystem));

      driverXbox.rightTrigger(0.05).whileTrue(Commands.run(() -> 
                                              {ShooterFlywheel.setBothDutyCycleSetpoint(
                                                  Math.max(-1, Math.min(1, driverXbox.getRightTriggerAxis())));
                                                System.out.println("Shooter set to " + driverXbox.getRightTriggerAxis());}, ShooterFlywheel));
          
      driverXbox.leftTrigger(0.05).whileTrue(Commands.run(() -> 
                                              {Conveyor.setDutyCycleSetpoint(
                                                  Math.max(-1, Math.min(1, driverXbox.getLeftTriggerAxis()))   );
                                                System.out.println("Conveyor set to " + driverXbox.getLeftTriggerAxis());}, Conveyor).finallyDo(() -> Conveyor.setDutyCycleSetpoint(0)));
                                            
      driverXbox.leftTrigger(0.05).whileTrue(Commands.run(() -> 
                                              {IntakeRollers.setDutyCycleSetpoint(
                                                  Math.max(-1, Math.min(1, 0.3)));
                                                System.out.println("IntakeRollers set to " + 0.2);
                                              }, IntakeRollers));
      // driverXbox.povRight().whileTrue(Commands.run(() -> 
      //                                         {IntakeArm.setVelocity(10);
      //                                           System.out.println("Intake Arm set to 10");}, IntakeArm));                                

      // driverXbox.povLeft().whileTrue(Commands.run(() -> 
      //                                         {IntakeArm.setVelocity(-10);
      //                                           System.out.println("Intake Arm set to -10");}, IntakeArm));
                                              
      driverXbox.x().whileTrue(IntakeArm.popOut(3));
       driverXbox.b().whileTrue(IntakeArm.popOut(-3));
      //driverXbox.b().whileTrue(;

      // AimHood hoodAim = new AimHood(HoodSubsystem, ShooterFlywheel, () -> 4);
      // driverXbox.a().whileTrue(hoodAim);

      // driverXbox.y().toggleOnTrue(new AlignSwerveCommand(
      //     drivebase,
      //     driveAngularVelocity,
      //     Robot.isSimulation()
      // ));

     
    //      var topRightOfBumper = new Pose2d().getTranslation();
    // var bottomLeftOfBumper = new Pose2d().getTranslation();
    // var BumperRight = new Rectangle2d(topRightOfBumper,bottomLeftOfBumper);


    //   var topBlueBumperTopLeft = new Translation2d(3.565, 8.016);
    //   var topBlueBumperBottomRight = new Translation2d(5.571, 4.618);
    //   var topBlueBumper = new Rectangle2d(topBlueBumperTopLeft, topBlueBumperBottomRight);

    //   var bottomBlueBumperTopLeft = new Translation2d(3.541, 3.379);
    //   var bottomBlueBumperBottomRight = new Translation2d(5.579, 1.544);
    //   var bottomBlueBumper = new Rectangle2d(bottomBlueBumperTopLeft, bottomBlueBumperBottomRight);

    //   var topRedBumperTopLeft = new Translation2d(10.982, 6.775);
    //   var topRedBumperBottomRight = new Translation2d(12.769, 4.586);
    //   var topRedBumper = new Rectangle2d(topRedBumperTopLeft, topRedBumperBottomRight);

    //   var bottomRedBumperTopLeft = new Translation2d(10.966, 3.411);
    //   var bottomRedBumpterBottomRight = new Translation2d(12.881, 1.673);
    //   var bottomRedBumper = new Rectangle2d(bottomRedBumperTopLeft, bottomRedBumpterBottomRight);

    //   Predicate<Pose2d> inTheBumpers = pose -> 
    //                                   topBlueBumper.contains(pose.getTranslation()) ||
    //                                   bottomBlueBumper.contains(pose.getTranslation()) ||
    //                                   topRedBumper.contains(pose.getTranslation()) ||
    //                                   bottomRedBumper.contains(pose.getTranslation())
    //                                   ;
                                  

    //   double headingDegrees = 45;
    
    //   SwerveInputStream stream = driveAngularVelocity.copy().withControllerHeadingAxis(
    //                                                           ()->  Math.cos(Degrees.of(headingDegrees).in(Radians)), 
    //                                                           ()->Math.sin(Degrees.of(headingDegrees).in(Radians)))
    //                                                         .headingWhile(true);
      
    //   Command driveAimedAtBumperFieldOriented = drivebase.driveFieldOriented(stream);


    //   Trigger Bumper = new Trigger( 
    //                               () -> inTheBumpers.test(drivebase.getPose())
    //                               );
    //                               //   .whileTrue(
    //                               //               Commands.run(
    //                               //                             ()-> {drivebase.setDefaultCommand(driveAimedAtBumperFieldOriented);} 
    //                               //                           )
    //                               //   .finallyDo(() -> drivebase.setDefaultCommand(drivebase.driveFieldOriented(driveAngularVelocity))
    //                               //   )
                                  // );

      //driverXbox.b().whileTrue(driveAimedAtBumperFieldOriented);

    

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    IntakeArm.popOut(0.4);
    
    Conveyor.setDefaultCommand(Commands.run(() -> Conveyor.setDutyCycle(0.5), Conveyor));

    return drivebase.getAutonomousCommand("mid start 1");
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }

  public SwerveSubsystem getSwerveSubsystem() {
    return drivebase;
  }
}
