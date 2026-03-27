// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.OperatorConstants;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IntakeArm;
import frc.robot.subsystems.IntakeRollers;
import frc.robot.subsystems.RollerSubsystem;
import frc.robot.subsystems.ShooterFlywheel;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;

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
      drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
    } else
    {
      System.out.println("Setting default command");
      drivebase.setDefaultCommand(testCommand);
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

    IntakeArm.setDefaultCommand(Commands.run(() -> IntakeArm.setVelocity(20), IntakeArm));

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

      driverXbox.povUp().whileTrue(Commands.run(() -> 
                                              {HoodSubsystem.setVelocity(10);
                                                System.out.println("Hood set to 10");}, HoodSubsystem));
      driverXbox.povDown().whileTrue(Commands.run(() -> 
                                                 {HoodSubsystem.setVelocity(-10);
                                                   System.out.println("Hood set to -10");}, HoodSubsystem));

      driverXbox.rightTrigger(0.05).whileTrue(Commands.run(() -> 
                                              {ShooterFlywheel.setBothDutyCycleSetpoint(
                                                  Math.max(-1, Math.min(1, driverXbox.getRightTriggerAxis())));
                                                System.out.println("Shooter set to " + driverXbox.getRightTriggerAxis());}, ShooterFlywheel));
          
      driverXbox.leftTrigger(0.05).whileTrue(Commands.run(() -> 
                                              {Conveyor.setDutyCycleSetpoint(
                                                  Math.max(-1, Math.min(1,-0.2)));
                                                System.out.println("Conveyor set to " + "-0.2");}, Conveyor));
                                            
      driverXbox.leftTrigger(0.05).whileTrue(Commands.run(() -> 
                                              {IntakeRollers.setDutyCycleSetpoint(
                                                  Math.max(-1, Math.min(1, driverXbox.getLeftTriggerAxis())));
                                                System.out.println("IntakeRollers set to " + driverXbox.getLeftTriggerAxis());}, IntakeRollers));
      // driverXbox.povRight().whileTrue(Commands.run(() -> 
      //                                         {IntakeArm.setVelocity(10);
      //                                           System.out.println("Intake Arm set to 10");}, IntakeArm));                                

      // driverXbox.povLeft().whileTrue(Commands.run(() -> 
      //                                         {IntakeArm.setVelocity(-10);
      //                                           System.out.println("Intake Arm set to -10");}, IntakeArm));
                                              
      driverXbox.x().whileTrue(IntakeArm.popOut());

      // driverXbox.povUp().onTrue(HoodSubsystem.setDegreeCommand(30));
      // driverXbox.povDown().onTrue(HoodSubsystem.setDegreeCommand(60));

    //      var topRightOfTrench = new Pose2d().getTranslation();
    // var bottomLeftOfTrench = new Pose2d().getTranslation();
    // var trenchRight = new Rectangle2d(topRightOfTrench,bottomLeftOfTrench);


    //   var topBlueTrenchTopLeft = new Translation2d(5.238, 8.016);
    //   var topBlueTrenchBottomRight = new Translation2d(4.048, 6.747);
    //   var topBlueTrench = new Rectangle2d(topBlueTrenchTopLeft, topBlueTrenchBottomRight);

    //   var bottomBlueTrenchTopLeft = new Translation2d(5.146, 1.409);
    //   var bottomBlueTrenchBottomRight = new Translation2d(4.034, 0.045);
    //   var bottomBlueTrench = new Rectangle2d(bottomBlueTrenchTopLeft, bottomBlueTrenchBottomRight);

    //   var topRedTrenchTopLeft = new Translation2d(12.558, 8.013);
    //   var topRedTrenchBottomRight = new Translation2d(11.394, 6.816);
    //   var topRedTrench = new Rectangle2d(topRedTrenchTopLeft, topRedTrenchBottomRight);

    //   var bottomRedTrenchTopLeft = new Translation2d(12.539, 1.254);
    //   var bottomRedTrenchBottomRight = new Translation2d(11.394, 0.045);
    //   var bottomRedTrench = new Rectangle2d(bottomRedTrenchTopLeft, bottomRedTrenchBottomRight);

    //   Predicate<Pose2d> inTheTrenches = pose -> 
    //                                   topBlueTrench.contains(pose.getTranslation()) ||
    //                                   bottomBlueTrench.contains(pose.getTranslation()) ||
    //                                   topRedTrench.contains(pose.getTranslation()) ||
    //                                   bottomRedTrench.contains(pose.getTranslation())
    //                                   ;
                                  

    //   double headingDegrees = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ? 0.0 : 180.0;
    
    //   SwerveInputStream stream = driveAngularVelocity.copy().withControllerHeadingAxis(
    //                                                           ()->  Math.cos(Degrees.of(headingDegrees).in(Radians)), 
    //                                                           ()->Math.sin(Degrees.of(headingDegrees).in(Radians)))
    //                                                         .headingWhile(true);
      
    //   Command driveAimedAtTrenchFieldOriented = drivebase.driveFieldOriented(stream);


    //   Trigger trench = new Trigger( 
    //                               () -> inTheTrenches.test(drivebase.getPose())
    //                               )
    //                                 .whileTrue(
    //                                             Commands.run(
    //                                                           ()-> {drivebase.setDefaultCommand(driveAimedAtTrenchFieldOriented);} 
    //                                                         )

    //                                 .finallyDo(  () -> drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity) )
    //                               );

    

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    return drivebase.getAutonomousCommand("New Auto");
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }

  public SwerveSubsystem getSwerveSubsystem() {
    return drivebase;
  }
}
