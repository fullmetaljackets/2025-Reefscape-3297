package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.LimelightHelpers;

public class AutoStrafeToCoral extends Command {
    private final CommandSwerveDrivetrain m_drivetrain;
    private final SwerveRequest.RobotCentric m_alignRequest;
    private final Limelight m_limelight;
    private final double kp_Strafe = 1.2;

    private final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric();

    public AutoStrafeToCoral(CommandSwerveDrivetrain drivetrain, Limelight limelight) {
        m_drivetrain = drivetrain;
        m_limelight = limelight;
        m_alignRequest = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.Velocity);
    }

    @Override
    public void initialize() {
        // Initialization code if needed
        LimelightHelpers.setPipelineIndex("limelight-intake", 0);
        LimelightHelpers.setLEDMode_ForceOn("limelight-intake");

    }

    @Override
    public void execute() {
        double angleError = -Units.degreesToRadians(LimelightHelpers.getTX("limelight-intake")); // Assume you have a method to get the angle error
        double strafeError = Math.tan(angleError);
        
        double strafeSpeed = kp_Strafe * strafeError;

        SmartDashboard.putNumber("strafe error", strafeError);
        SmartDashboard.putNumber("strafe speed", strafeSpeed);

        // Drive the robot

        m_drivetrain.setControl(
        m_alignRequest.withVelocityX(0) // Drive forward with negative Y (forward)
            .withVelocityY(strafeSpeed ) // Drive left with negative X (left)
            .withRotationalRate(0) // Drive counterclockwise with negative X (left)
        );
    }

    @Override
    public boolean isFinished() {
        double angleError = -Units.degreesToRadians(LimelightHelpers.getTX("limelight-intake")); // Assume you have a method to get the angle error
        double strafeError = Math.tan(angleError);

        double strafeSpeed = kp_Strafe * strafeError;


        // Define a condition to end the command, e.g., when the robot is close enough to the tag
        return Math.abs(strafeSpeed) < 0.06; 
        // return false;
    }

    @Override
    public void end(boolean interrupted) {
        LimelightHelpers.setLEDMode_ForceOff("limelight-intake");

        // Stop the drivetrain when the command ends
        m_drivetrain.setControl(
        drive.withVelocityX(0) // Drive forward with negative Y (forward)
            .withVelocityY(0) // Drive left with negative X (left)
            .withRotationalRate(0)); // Drive counterclockwise with negative X (left)
        // );
    }
}