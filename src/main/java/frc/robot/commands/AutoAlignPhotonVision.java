package frc.robot.commands;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AutoAlignPhotonVision extends Command {
    private final CommandSwerveDrivetrain m_drivetrain;
    private final SwerveRequest.RobotCentric m_alignRequest;


    public AutoAlignPhotonVision(CommandSwerveDrivetrain drivetrain) {
        m_drivetrain = drivetrain;
        m_alignRequest = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.Velocity);
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        // Initialization code if needed

    }

    @Override
    public void execute() {
        PhotonCamera camera = new PhotonCamera("Arducam_OV9281(Apriltag)");

        PhotonPipelineResult result = camera.getLatestResult();
        PhotonTrackedTarget target = result.getBestTarget();
    
        double yaw =  target.getYaw();
        double pitch = target.getPitch();
        int targetID = target.getFiducialId();
        double poseAmbiguity = target.getPoseAmbiguity();

        SmartDashboard.putNumber("Yaw", yaw);
        SmartDashboard.putNumber("Pitch", pitch);
        SmartDashboard.putNumber("Target ID", targetID);
        SmartDashboard.putNumber("Pose Ambiguity", poseAmbiguity);
    }

}
