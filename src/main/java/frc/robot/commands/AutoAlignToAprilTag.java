package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.LimelightHelpers;

public class AutoAlignToAprilTag extends Command {
    private final CommandSwerveDrivetrain m_drivetrain;
    private final SwerveRequest.RobotCentric m_alignRequest;
    private final Limelight m_limelight;
    private final double kP_Distance = 0.1; // Proportional control constant
    private final double kp_Strafe = 0.2;
    private final double kp_Angle = 3;

    private final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric();

    public AutoAlignToAprilTag(CommandSwerveDrivetrain drivetrain, Limelight limelight) {
        m_drivetrain = drivetrain;
        m_limelight = limelight;
        m_alignRequest = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.Velocity);
        addRequirements( limelight);
    }

    @Override
    public void initialize() {
        // Initialization code if needed
    }

    @Override
    public void execute() {
        LimelightHelpers.setPipelineIndex("limelight", 0);
        
        double distance = m_limelight.getDistanceToReef();
        double angleError = -Units.degreesToRadians(LimelightHelpers.getTX("limelight")); // Assume you have a method to get the angle error
        double strafeError = distance * Math.tan(angleError);
        
        // Proportional control for distance and angle
        double forwardSpeed = kP_Distance * distance;
        double turnSpeed = kp_Angle * angleError;
        double strafeSpeed = kp_Strafe * strafeError;

        SmartDashboard.putNumber("forward speed", forwardSpeed);
        SmartDashboard.putNumber("turn speed", turnSpeed);
        SmartDashboard.putNumber("strafe speed", strafeSpeed);

        // Drive the robot
        // drivetrain.arcadeDrive(forwardSpeed, turnSpeed);

        // drivetrain.applyRequest(() ->
        // drive.withVelocityX(forwardSpeed) // Drive forward with negative Y (forward)
        //     .withVelocityY(strafeSpeed) // Drive left with negative X (left)
        //     .withRotationalRate(turnSpeed) // Drive counterclockwise with negative X (left)
        // );

        m_drivetrain.setControl(
        m_alignRequest.withVelocityX(0) // Drive forward with negative Y (forward)
            .withVelocityY(-strafeSpeed) // Drive left with negative X (left)
            .withRotationalRate(turnSpeed) // Drive counterclockwise with negative X (left)
        );
 
        // drivetrain.setControl(
        //     m_alignRequest
        //         .withVelocityX(forwardSpeed)
        //         .withVelocityY(strafeSpeed)
        //         .withRotationalRate(turnSpeed));
    }

    @Override
    public boolean isFinished() {
        double distance = m_limelight.getDistanceToReef();
        double angleError = -Units.degreesToRadians(LimelightHelpers.getTX("limelight"));
        double strafeError = distance * Math.tan(angleError);

        // Define a condition to end the command, e.g., when the robot is close enough to the tag
        return Math.abs(distance) < 0.1 && Math.abs(angleError) < 0.1 && Math.abs(strafeError) < 0.1;
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the drivetrain when the command ends
        // drivetrain.arcadeDrive(0, 0);
        // drivetrain.applyRequest(() ->
        // drive.withVelocityX(0) // Drive forward with negative Y (forward)
        //     .withVelocityY(0) // Drive left with negative X (left)
        //     .withRotationalRate(0) // Drive counterclockwise with negative X (left)
        // );
    }
}