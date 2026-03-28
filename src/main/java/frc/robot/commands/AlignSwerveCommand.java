package frc.robot.commands;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.SwerveSubsystem;
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

    private final SimpleMotorFeedforward realFeedFoward;
    private final SimpleMotorFeedforward simFeedFoward;

    private boolean simulation;

    private final double targetHeight = 1.5;
    private final Translation3d targetPos = new Translation3d(11.95, 4.05, targetHeight);

    public AlignSwerveCommand(SwerveSubsystem drivebase, SwerveInputStream inputStream, boolean simulation) {
        this.drivebase = drivebase;
        this.inputStream = inputStream;
        this.simulation = simulation;
        addRequirements(drivebase);

        // its awful
        //alignPID = new PIDController(0.01, 0, 0.00575);
        alignPID = new PIDController(0.3, 0, 0.05);

        this.realFeedFoward = new SimpleMotorFeedforward(0.5, 0, 0); 
        this.simFeedFoward = new SimpleMotorFeedforward(0.5, 0.1, 0);
        
        alignPID.setTolerance(0.1);
        alignPID.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void execute() {
        ChassisSpeeds driverSpeeds = inputStream.get();

        Pose2d robotPose;
        double latency;

        if (!simulation && LimelightHelpers.getTV(Constants.limelightName)) {
            robotPose = LimelightHelpers.getBotPose3d_wpiBlue(Constants.limelightName).toPose2d();
            latency = LimelightHelpers.getLatency_Pipeline(Constants.limelightName) / 1000.0 + LimelightHelpers.getLatency_Capture(Constants.limelightName) / 1000.0;
        } else {
            robotPose = drivebase.getPose();
            latency = 0.15;
        }
        

        if (robotPose == null) {
            System.out.println("LIMELIGHT IS NOT WORKING!!! or sim is not working?");
            return;
        }

        ChassisSpeeds fieldSpeeds = drivebase.getFieldVelocity();

        double predictedX = robotPose.getX() + (fieldSpeeds.vxMetersPerSecond * latency); // 0.15 seconds
        double predictedY = robotPose.getY() + (fieldSpeeds.vyMetersPerSecond * latency);

        double dx = targetPos.getX() - predictedX;
        double dy = targetPos.getY() - predictedY;

        ///////// BE VERY CAREFUL, THE OFFSET COULD BE WRONG, ADJUST AS NEEDED /////////
        double targetAngle = Math.atan2(dy, dx); //+ (Math.PI / 2.0);
        double headingError = targetAngle - robotPose.getRotation().getRadians();
        headingError = MathUtil.inputModulus(headingError, -Math.PI, Math.PI);

        double angularSpeed = alignPID.calculate(headingError, 0);

        SimpleMotorFeedforward feedFoward = simulation ? simFeedFoward : realFeedFoward;
        double feedFowardOutput = feedFoward.calculate(angularSpeed); 
        double totalAngularSpeed = angularSpeed + feedFowardOutput;

        drivebase.drive(
            new Translation2d(driverSpeeds.vxMetersPerSecond, driverSpeeds.vyMetersPerSecond),
            totalAngularSpeed,
            true
        );

        SmartDashboard.putNumber("Align TX", headingError);
        SmartDashboard.putNumber("Angular Speed", angularSpeed);
    }

    @Override
    public boolean isFinished()
    {
        return alignPID.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        drivebase.drive(new Translation2d(0, 0), 0, true);
    }
}
