//  /** A class that holds various triggers for control logic. */
//  package frc.robot.utils;

//  import static edu.wpi.first.units.Units.Degrees;
// import static edu.wpi.first.units.Units.Inches;
// import static edu.wpi.first.units.Units.Meters;
// import static edu.wpi.first.units.Units.RPM;
// import static edu.wpi.first.units.Units.Rotations;

// import edu.wpi.first.math.Pair;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Pose3d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Rotation3d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.geometry.Translation3d;
// import edu.wpi.first.units.measure.Angle;
// import edu.wpi.first.units.measure.AngularVelocity;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.wpilibj2.command.WaitCommand;
// import edu.wpi.first.wpilibj2.command.button.Trigger;

// // import frc.robot.subsystems.ClimberSubsystem;
// // import frc.robot.subsystems.BottomIntakeSubsystem;

// import frc.robot.subsystems.SwerveSubsystem;

// import frc.robot.utils.AllianceFlipUtil;
// import frc.robot.constants.FieldConstants;
// import frc.robot.utils.ZoneTrigger;
// import java.util.function.Supplier;

//   public  class CustomTriggers {
//     public static ZoneTrigger bumpZone =
//         new ZoneTrigger(
//             "Bump",
//             Pair.of(new Translation2d(3.75, 1.5), new Translation2d(5.5, 3.5)),
//             Pair.of(new Translation2d(3.75, 4.5), new Translation2d(5.5, 6.5)),
//             Pair.of(new Translation2d(11, 4.5), new Translation2d(12.75, 6.5)),
//             Pair.of(new Translation2d(11, 1.5), new Translation2d(12.75, 3.5)));

//     public static ZoneTrigger scoringZone =
//         new ZoneTrigger(
//             "Scoring", Pair.of(new Translation2d(1.5, 0.5), new Translation2d(3.5, 7.5)));

//     public static ZoneTrigger leftNeutralZone =
//         new ZoneTrigger(
//             "Left Neutral",
//             Pair.of(
//                 new Translation2d(
//                     FieldConstants.LinesVertical.starting, FieldConstants.LinesHorizontal.center),
//                 new Translation2d(
//                     FieldConstants.LinesVertical.oppAllianceZone, FieldConstants.fieldWidth)));

//     public static ZoneTrigger rightNeutralZone =
//         new ZoneTrigger(
//             "Right Neutral",
//             Pair.of(
//                 new Translation2d(FieldConstants.LinesVertical.starting, 0),
//                 new Translation2d(
//                     FieldConstants.LinesVertical.oppAllianceZone,
//                     FieldConstants.LinesHorizontal.center)));
//   }
