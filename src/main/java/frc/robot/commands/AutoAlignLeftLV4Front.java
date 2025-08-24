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

public class AutoAlignLeftLV4Front extends Command {
    private final CommandSwerveDrivetrain m_drivetrain;
    private final SwerveRequest.RobotCentric m_alignRequest;
    private final Limelight m_limelight;
    private final double kP_Distance = 0.05; // Proportional control constant
    private final double DistanceOffset = 17.5;
    private final double kp_Strafe = 2;
    private final double kp_Angle = 1.8;

    // private final CommandXboxController DriveStick = new CommandXboxController(0);

    private final Pose3d botPose = LimelightHelpers.getBotPose3d("limelight-sone");
    private final Pose3d targetPose = LimelightHelpers.getTargetPose3d_RobotSpace("limelight-sone");

    private final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric();

    public AutoAlignLeftLV4Front(CommandSwerveDrivetrain drivetrain, Limelight limelight) {
        m_drivetrain = drivetrain;
        m_limelight = limelight;
        m_alignRequest = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.Velocity);
        addRequirements( limelight);
    }

    @Override
    public void initialize() {
        // Initialization code if needed
        LimelightHelpers.setPipelineIndex("limelight-sone", 1);

    }

    @Override
    public void execute() {
        // LimelightHelpers.getTargetPose3d_CameraSpace("limelight");
        
        double distance = m_limelight.getDistanceToReef() - DistanceOffset;
        double angleError = -Units.degreesToRadians(LimelightHelpers.getTX("limelight-sone")); // Assume you have a method to get the angle error
        double strafeError = Math.tan(angleError);
        
        // Proportional control for distance and angle
        double forwardSpeed = kP_Distance * distance;
        double turnSpeed = kp_Angle * angleError;
        double strafeSpeed = kp_Strafe * strafeError;

        SmartDashboard.putNumber("strafe error", strafeError);
        SmartDashboard.putNumber("distance", distance);

        SmartDashboard.putNumber("forward speed", forwardSpeed);
        SmartDashboard.putNumber("turn speed", turnSpeed);
        SmartDashboard.putNumber("strafe speed", strafeSpeed);

        // Drive the robot
        // drivetrain.arcadeDrive(forwardSpeed, turnSpeed);


        m_drivetrain.setControl(
        m_alignRequest.withVelocityX(forwardSpeed ) // Drive forward with negative Y (forward)
            .withVelocityY(strafeSpeed ) // Drive left with negative X (left)
            .withRotationalRate(0) // Drive counterclockwise with negative X (left)
        );
    }

    @Override
    public boolean isFinished() {
        double distance = m_limelight.getDistanceToReef() - DistanceOffset;
        double angleError = -Units.degreesToRadians(LimelightHelpers.getTX("limelight-sone")); // Assume you have a method to get the angle error
        double strafeError = Math.tan(angleError);

        double forwardSpeed = kP_Distance * distance;
        double turnSpeed = kp_Angle * angleError;
        double strafeSpeed = kp_Strafe * strafeError;


        // Define a condition to end the command, e.g., when the robot is close enough to the tag
        return Math.abs(forwardSpeed) < 0.05 
        // && Math.abs(turnSpeed) < 0.1;
        && Math.abs(strafeSpeed) < 0.08;
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the drivetrain when the command ends
        // m_drivetrain.setControl(
        // drive.withVelocityX(0) // Drive forward with negative Y (forward)
        //     .withVelocityY(0) // Drive left with negative X (left)
        //     .withRotationalRate(0)); // Drive counterclockwise with negative X (left)
        // // );
    }
}