package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.FakeLimelight;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveInputStream;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

public class AlignSwerveCommandWithPhoton extends Command {

    private final SwerveSubsystem drivebase;
    private final SwerveInputStream inputStream;
    private final PIDController alignPID;
    private final boolean simulation;
    private final PhotonCamera camera;

    public AlignSwerveCommandWithPhoton(
            SwerveSubsystem drivebase,
            SwerveInputStream inputStream,
            boolean simulation) {

        this.drivebase = drivebase;
        this.inputStream = inputStream;
        this.simulation = simulation;
        this.camera = new PhotonCamera(Constants.photonCameraName);

        addRequirements(drivebase);

        alignPID = new PIDController(0.01, 0, 0.005);
        alignPID.setTolerance(1);
    }

    @Override
    public void initialize() {
        alignPID.reset();
    }

    @Override
    public void execute() {
        ChassisSpeeds driverSpeeds = inputStream.get();

        if (simulation) {
            double tx = FakeLimelight.getTX();
            runAlignment(driverSpeeds, tx);
            return;
        }

        var result = camera.getLatestResult();

        if (!result.hasTargets()) {
            drivebase.drive(
                driverSpeeds
            );
            SmartDashboard.putBoolean("Photon Has Target", false);
            return;
        }

        PhotonTrackedTarget target = result.getBestTarget();
        double yaw = target.getYaw();

        runAlignment(driverSpeeds, yaw);
    }

    private void runAlignment(ChassisSpeeds driverSpeeds, double yaw) {
        double angularSpeed = alignPID.calculate(yaw, 0);

        drivebase.drive(
            new Translation2d(driverSpeeds.vxMetersPerSecond, driverSpeeds.vyMetersPerSecond),
            angularSpeed,
            true
        );

        SmartDashboard.putBoolean("Photon Has Target", true);
        SmartDashboard.putNumber("Photon Yaw", yaw);
        SmartDashboard.putNumber("Angular Speed", angularSpeed);
        SmartDashboard.putBoolean("Align PID At Setpoint", alignPID.atSetpoint());
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        drivebase.drive(new Translation2d(0, 0), 0, true);
    }
}