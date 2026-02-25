// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import static edu.wpi.first.units.Units.*;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be
 * declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */

// nikhilsai swerve constants

// {
// "drive": {
// "p": 1.5,
// "i": 0.04,
// "d": 0.2,
// "f": 0.5,
// "iz": 0
// },
// "angle": {
// "p": 0.8,
// "i": 0.01,
// "d": 1,
// "f": 0.4,
// "iz": 0
// }
// }

public final class Constants {

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME = 0.13; // s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED = Units.feetToMeters(14.5);
  // Maximum speed of the robot in meters per second, used to limit acceleration.

  // public static final class AutonConstants
  // {
  //
  // public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0,
  // 0);
  // public static final PIDConstants ANGLE_PID = new PIDConstants(0.4, 0, 0.01);
  // }

  public static final class DrivebaseConstants {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants {

    // Joystick Deadband
    public static final double DEADBAND = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT = 6;
  }

  public static final class HoodConstants {
    // CAN IDs — must not clash with swerve (5,10,12,20,22,36,37,45)
    public static final int HOOD_LEADER_ID = 30;
    public static final int HOOD_FOLLOWER_ID = 31;

    // PID gains (tune these on the real robot)
    public static final double kP = 4.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    // Trapezoidal motion profile constraints
    public static final AngularVelocity MAX_VELOCITY = DegreesPerSecond.of(180);
    public static final AngularAcceleration MAX_ACCELERATION = DegreesPerSecondPerSecond.of(90);

    // Soft limits (sent to motor controller)
    public static final Angle SOFT_LIMIT_LOWER = Degrees.of(-5);
    public static final Angle SOFT_LIMIT_UPPER = Degrees.of(60);

    // Hard limits (simulation physical stops)
    public static final Angle HARD_LIMIT_LOWER = Degrees.of(-10);
    public static final Angle HARD_LIMIT_UPPER = Degrees.of(65);

    // Gear ratio (motor rotations : mechanism rotations)
    public static final double GEAR_RATIO = 25.0;

    // Physical parameters for simulation
    public static final Distance ARM_LENGTH = Meters.of(0.3);
    public static final Mass ARM_MASS = Kilograms.of(2.0);

    // Starting position
    public static final Angle STARTING_POSITION = Degrees.of(0);

    // Current limit
    public static final Current STATOR_CURRENT_LIMIT = Amps.of(40);
  }
}
