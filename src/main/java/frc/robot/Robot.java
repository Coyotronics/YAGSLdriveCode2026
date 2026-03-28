// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.simulation.Simulation;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to each mode, as
 * described in the TimedRobot documentation. If you change the name of this class or the package after creating this
 * project, you must also update the build.gradle file in the project.
 */
public class Robot extends TimedRobot
{

  private static Robot   instance;
  private        Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private Timer disabledTimer;

  public Robot()
  {
    instance = this;
  }

  public static Robot getInstance()
  {
    return instance;
  }

  /**
   * This function is run when the robot is first started up and should be used for any initialization code.
   */

  // public final Field2d m_field = new Field2d();

  private final double targetHeight = 1.5;
  private final Translation3d targetPos = new Translation3d(11.9, 4.05, targetHeight);
  private StructPublisher<Pose3d> targetPublisher;

  private StructPublisher<Pose3d> posePublisher;
  private StructArrayPublisher<Pose3d> redGoalPublisher;


  @Override
  public void robotInit()
  {



    // Do this in either robot or subsystem init
    // SmartDashboard.putData("Field", m_field);

    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    // Create a timer to disable motor brake a few seconds after disable.  This will let the robot stop
    // immediately when disabled, but then also let it be pushed more 
    disabledTimer = new Timer();

    posePublisher = NetworkTableInstance.getDefault().getStructTopic("Sim/My Pose", Pose3d.struct).publish();

    if (isSimulation()) {
        Simulation.init();

        targetPublisher = NetworkTableInstance.getDefault().getStructTopic("Sim/Target Pose", Pose3d.struct).publish();
        targetPublisher.set(new Pose3d(targetPos.getX(), targetPos.getY(), targetPos.getZ(), new Rotation3d()));

        redGoalPublisher = NetworkTableInstance.getDefault().getStructArrayTopic("/Field/Red Goal Points", Pose3d.struct).publish();

        DriverStation.silenceJoystickConnectionWarning(true);
    }
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics that you want ran
   * during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic()
  {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    SmartDashboard.putNumber("Robot Heading", m_robotContainer.getSwerveSubsystem().getHeading().getDegrees());
  
    if (isSimulation()) {
      Simulation.updateFakeLimelight(targetPos, m_robotContainer.getSwerveSubsystem());

      // too lazy to make it better rn
      double heightHalf = 0.541 / 2;
      double widthHalf = 0.686 / 2;
      double lengthHalf = 0.504 / 2;

      Translation3d[] points = new Translation3d[8];

      points[0] = new Translation3d(lengthHalf, widthHalf, 2 * heightHalf);
      points[1] = new Translation3d(lengthHalf, widthHalf, 0);
      points[2] = new Translation3d(lengthHalf, -widthHalf, 2 * heightHalf);
      points[3] = new Translation3d(lengthHalf, -widthHalf, 0);
      points[4] = new Translation3d(-lengthHalf, widthHalf, 2 * heightHalf);
      points[5] = new Translation3d(-lengthHalf, widthHalf, 0);
      points[6] = new Translation3d(-lengthHalf, -widthHalf, 2 * heightHalf);
      points[7] = new Translation3d(-lengthHalf, -widthHalf, 0);

      Translation3d[] intakePoints = new Translation3d[8];

      intakePoints[0] = new Translation3d(widthHalf, lengthHalf, 2 * heightHalf);
      intakePoints[1] = new Translation3d(widthHalf, lengthHalf,  0);
      intakePoints[2] = new Translation3d(widthHalf + 0.25, lengthHalf, 2 * heightHalf);
      intakePoints[3] = new Translation3d(widthHalf + 0.25, lengthHalf, 0);
      intakePoints[4] = new Translation3d(widthHalf, -lengthHalf, 2 * heightHalf);
      intakePoints[5] = new Translation3d(widthHalf, -lengthHalf, 0);
      intakePoints[6] = new Translation3d(widthHalf + 0.25, -lengthHalf, 2 * heightHalf);
      intakePoints[7] = new Translation3d(widthHalf + 0.25, -lengthHalf, 0);

      Translation3d redGoalPos = new Translation3d(11.90, 4.05, 0);

      double goalWidth  = 1;
      double goalLength = 1;
      double goalHeight = 1.5;

      Translation3d[] redGoalPoints = new Translation3d[8];
      
      double halfWidth  = goalWidth / 2.0;
      double halfLength = goalLength / 2.0;
      double halfHeight = goalHeight / 2.0;

      redGoalPoints[0] = new Translation3d(redGoalPos.getX() + halfLength, redGoalPos.getY() + halfWidth, redGoalPos.getZ() + goalHeight);
      redGoalPoints[1] = new Translation3d(redGoalPos.getX() + halfLength, redGoalPos.getY() - halfWidth, redGoalPos.getZ() + goalHeight);
      redGoalPoints[2] = new Translation3d(redGoalPos.getX() - halfLength, redGoalPos.getY() + halfWidth, redGoalPos.getZ() + goalHeight);
      redGoalPoints[3] = new Translation3d(redGoalPos.getX() - halfLength, redGoalPos.getY() - halfWidth, redGoalPos.getZ() + goalHeight);
      redGoalPoints[4] = new Translation3d(redGoalPos.getX() + halfLength, redGoalPos.getY() + halfWidth, redGoalPos.getZ());
      redGoalPoints[5] = new Translation3d(redGoalPos.getX() + halfLength, redGoalPos.getY() - halfWidth, redGoalPos.getZ());
      redGoalPoints[6] = new Translation3d(redGoalPos.getX() - halfLength, redGoalPos.getY() + halfWidth, redGoalPos.getZ());
      redGoalPoints[7] = new Translation3d(redGoalPos.getX() - halfLength, redGoalPos.getY() - halfWidth, redGoalPos.getZ());

      Pose3d[] redGoalPoses = new Pose3d[redGoalPoints.length];
      for (int i = 0; i < redGoalPoints.length; i++) {
          redGoalPoses[i] = new Pose3d(redGoalPoints[i], new Rotation3d());
      }

      redGoalPublisher.set(redGoalPoses);

      Pose2d robotPose = m_robotContainer.getSwerveSubsystem().getSwerveDrive()
        .getSimulationDriveTrainPose()
        .orElse(m_robotContainer.getSwerveSubsystem().getPose());
      
      for (int i = 0; i < points.length; i++) {
          SmartDashboard.putNumber("Point" + i + " X", points[i].getX() + robotPose.getX());
          SmartDashboard.putNumber("Point" + i + " Y", points[i].getY() + robotPose.getY());
          SmartDashboard.putNumber("Point" + i + " Z", points[i].getZ()); 
      }

      Pose3d robotOrigin3d = new Pose3d(
          robotPose.getX(),
          robotPose.getY(),
          0,
          new Rotation3d(robotPose.getRotation())
      );

      Simulation.updateBalls(points, intakePoints, robotOrigin3d);
      Simulation.updateVertexPositionsAdvantageScope(points, intakePoints, robotOrigin3d);
      Simulation.updateBallPositionsAdvantageScope();

   
      posePublisher.set(robotOrigin3d);
    }

    CommandScheduler.getInstance().run();

    //System.out.println("Everything zerox2");
    
    // m_field.setRobotPose(m_robotContainer.drivebase.getPose());
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit()
  {
    m_robotContainer.setMotorBrake(true);
    disabledTimer.reset();
    disabledTimer.start();
  }

  @Override
  public void disabledPeriodic()
  {
    if (disabledTimer.hasElapsed(Constants.DrivebaseConstants.WHEEL_LOCK_TIME))
    {
      m_robotContainer.setMotorBrake(false);
      disabledTimer.stop();
      disabledTimer.reset();
    }
  }

  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit()
  {
    m_robotContainer.setMotorBrake(true);
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null)
    {
      m_autonomousCommand.schedule();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic()
  {
  }

  @Override
  public void teleopInit()
  {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null)
    {
      m_autonomousCommand.cancel();
    } else
    {
      CommandScheduler.getInstance().cancelAll();
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic()
  {
  }

  @Override
  public void testInit()
  {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }


}
