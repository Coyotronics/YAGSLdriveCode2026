package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.stream.DoubleStream.DoubleMapMultiConsumer;

import org.opencv.video.TrackerDaSiamRPN;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.FakeLimelight;
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

    public AlignSwerveCommand(SwerveSubsystem drivebase, SwerveInputStream inputStream, boolean simulation) {
        this.drivebase = drivebase;
        this.inputStream = inputStream;
        this.simulation = simulation;
        addRequirements(drivebase);

        alignPID = new PIDController(0.01, 0, 0.005);
        alignPID.setTolerance(1);
    }

    @Override
    public void execute() {
        ChassisSpeeds driverSpeeds = inputStream.get();

        double tx = simulation ? FakeLimelight.getTX() : LimelightHelpers.getTX(Constants.limelightName);
        double angularSpeed = alignPID.calculate(tx, 0);
        
        drivebase.drive(
            new Translation2d(driverSpeeds.vxMetersPerSecond, driverSpeeds.vyMetersPerSecond),
            angularSpeed,
            true
        );
        
        SmartDashboard.putNumber("Limelight TX", tx);
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
