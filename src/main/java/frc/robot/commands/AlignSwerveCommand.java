package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveInputStream;
/*
import frc.robot.constants.ShotingOnTheFlyConstants;
import frc.robot.systems.ScoringSystem;
import frc.robot.utils.field.AllianceFlipUtil;
import frc.robot.utils.field.FieldConstants;
import frc.robot.utils.field.GeomUtil;
*/

//import java.util.function.Supplier;
//import lombok.experimental.ExtensionMethod;

public class AlignSwerveCommand extends Command {
    private final SwerveSubsystem drivebase;

    private final SwerveInputStream inputStream;

    private PIDController alignPID;

    private boolean simulation;

    private final double targetHeight = 1.5;
    private final Translation3d targetPos = new Translation3d(11.90, 4.05, targetHeight);

    public AlignSwerveCommand(SwerveSubsystem drivebase, SwerveInputStream inputStream, boolean simulation) {
        this.drivebase = drivebase;
        this.inputStream = inputStream;
        this.simulation = simulation;
        addRequirements(drivebase);

        // its awful
        alignPID = new PIDController(0.01, 0, 0.00575);
        
        alignPID.setTolerance(1);
    }

    @Override
    public void execute() {
        ChassisSpeeds driverSpeeds = inputStream.get();

        Pose2d robotPose;
        if (!simulation && LimelightHelpers.getTV(Constants.limelightName)) robotPose = LimelightHelpers.getBotPose3d_wpiBlue(Constants.limelightName).toPose2d();
        else robotPose = drivebase.getPose();

        if (robotPose == null) {
            System.out.println("LIMELIGHT IS NOT WORKING!!! or sim is not working?");
            return;
        }

        ChassisSpeeds fieldSpeeds = drivebase.getFieldVelocity();

        double dx = targetPos.getX() - robotPose.getX() - fieldSpeeds.vxMetersPerSecond;
        double dy = targetPos.getY() - robotPose.getY() - fieldSpeeds.vyMetersPerSecond;

        double targetAngle = Math.atan2(dy, dx) + (Math.PI / 2.0);
        double headingError  = Math.toDegrees(targetAngle - robotPose.getRotation().getRadians());
        headingError  = edu.wpi.first.math.MathUtil.inputModulus(headingError, -180, 180);

        double angularSpeed = alignPID.calculate(headingError, 0);

        drivebase.drive(
            new Translation2d(driverSpeeds.vxMetersPerSecond, driverSpeeds.vyMetersPerSecond),
            angularSpeed,
            true
        );

        SmartDashboard.putNumber("Align TX", headingError);
        SmartDashboard.putNumber("Angular Speed", angularSpeed);
    }

    @Override
    public boolean isFinished()
    {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        drivebase.drive(new Translation2d(0, 0), 0, true);
    }
}
