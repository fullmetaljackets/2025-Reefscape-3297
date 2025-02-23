package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.LimelightHelpers;

public class AutoAlignToAprilTag2 extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final Limelight limelight;
    private final SwerveRequest.RobotCentric m_alignRequest;
    private final double kP_Translation = 0.1; // Proportional control constant for translation
    private final double kP_Rotation = 0.1; // Proportional control constant for rotation

    public AutoAlignToAprilTag2(CommandSwerveDrivetrain drivetrain, Limelight limelight) {
        this.drivetrain = drivetrain;
        this.limelight = limelight;
                m_alignRequest = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.Velocity);

        addRequirements(drivetrain, limelight);
    }

    @Override
    public void initialize() {
        // Initialization code if needed
    }

    @Override
    public void execute() {
        // Retrieve the robot's current pose
        Pose3d robotPose = LimelightHelpers.getBotPose3d("limelight");

        // Retrieve the target pose relative to the robot
        Pose3d targetPose = LimelightHelpers.getTargetPose3d_RobotSpace("limelight");

        // Calculate the translation error
        Translation3d translationError = targetPose.getTranslation().minus(robotPose.getTranslation());

        // Calculate the rotation error
        Rotation3d rotationError = targetPose.getRotation().minus(robotPose.getRotation());

        // Proportional control for translation and rotation
        double forwardSpeed = kP_Translation * translationError.getX();
        double strafeSpeed = kP_Translation * translationError.getY();
        double turnSpeed = kP_Rotation * Units.radiansToDegrees(rotationError.getZ());

        //put robot pose on smart dashboard
        SmartDashboard.putNumber("robotX", robotPose.getX());
        SmartDashboard.putNumber("robotY", robotPose.getY());
        SmartDashboard.putNumber("robotrotation", Units.radiansToDegrees(robotPose.getRotation().getZ()));

        //put target pose on smart dashboard
        SmartDashboard.putNumber("targetX", targetPose.getX());
        SmartDashboard.putNumber("targetY", targetPose.getY());
        SmartDashboard.putNumber("targetrotation", Units.radiansToDegrees(targetPose.getRotation().getZ()));

        //put pose diference on smart dashboard
        SmartDashboard.putNumber("errorX", translationError.getX());
        SmartDashboard.putNumber("errorY", translationError.getY());
        SmartDashboard.putNumber("errorrotation", Units.radiansToDegrees(rotationError.getZ()));

        //put speeds on samrt dashboard for testing 
        SmartDashboard.putNumber("forward speed", forwardSpeed);
        SmartDashboard.putNumber("turn speed", turnSpeed);
        SmartDashboard.putNumber("strafe speed", strafeSpeed);

        // Apply the swerve drive request to the drivetrain

        drivetrain.setControl(
            m_alignRequest.withVelocityX(forwardSpeed) // Drive forward with negative Y (forward)
                .withVelocityY(strafeSpeed) // Drive left with negative X (left)
                .withRotationalRate(turnSpeed) // Drive counterclockwise with negative X (left)
            );
    
        // drivetrain.applyRequest(() ->
        //     new SwerveRequest.RobotCentric()
        //         .withVelocityX(forwardSpeed)
        //         .withVelocityY(strafeSpeed)
        //         .withRotationalRate(turnSpeed)
        // );
    }

    @Override
    public boolean isFinished() {
        // Define a condition to end the command, e.g., when the robot is close enough to the tag
        Pose3d robotPose = LimelightHelpers.getBotPose3d("limelight");
        Pose3d targetPose = LimelightHelpers.getTargetPose3d_RobotSpace("limelight");

        Translation3d translationError = targetPose.getTranslation().minus(robotPose.getTranslation());
        Rotation3d rotationError = targetPose.getRotation().minus(robotPose.getRotation());

        return translationError.getNorm() < 0.1 && Math.abs(rotationError.getZ()) < Units.degreesToRadians(5);
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the drivetrain when the command ends
        drivetrain.setControl(
            m_alignRequest.withVelocityX(0) // Drive forward with negative Y (forward)
                .withVelocityY(0) // Drive left with negative X (left)
                .withRotationalRate(0) // Drive counterclockwise with negative X (left)
            );

        // drivetrain.applyRequest(() ->
        //     new SwerveRequest.RobotCentric()
        //         .withVelocityX(0)
        //         .withVelocityY(0)
        //         .withRotationalRate(0)
        // );
    }
}