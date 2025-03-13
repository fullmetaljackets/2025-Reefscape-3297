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

public class AutoDriveToCoral extends Command {
    private final CommandSwerveDrivetrain m_drivetrain;
    private final SwerveRequest.RobotCentric m_alignRequest;
    private final Limelight m_limelight;
    private final double kP_Distance = 0.04; // Proportional control constant
    private final double kp_Strafe = 2.2;
    private final double kp_Angle = 2.2;
    private final double DistanceOffset = 0;
    private final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric();

    public AutoDriveToCoral(CommandSwerveDrivetrain drivetrain, Limelight limelight) {
        m_drivetrain = drivetrain;
        m_limelight = limelight;
        m_alignRequest = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.Velocity);
    }

    @Override
    public void initialize() {
        // Initialization code if needed
        LimelightHelpers.setPipelineIndex("limelight-intake", 0);

    }

    @Override
    public void execute() {
        
        double distance = LimelightHelpers.getTY("limelight-intake");
        double angleError = -Units.degreesToRadians(LimelightHelpers.getTX("limelight-intake")); // Assume you have a method to get the angle error


        double forwardSpeed = kP_Distance * distance;
        double turnSpeed = kp_Angle * angleError;

        SmartDashboard.putNumber("intake LL distance", distance);
        SmartDashboard.putNumber("intake LL forward speed", forwardSpeed);

        SmartDashboard.putNumber("intake LL angle error", angleError);
        SmartDashboard.putNumber("intake LL angle speed", angleError);

        m_drivetrain.setControl(
        m_alignRequest.withVelocityX(-forwardSpeed) // Drive forward with negative Y (forward)
            .withVelocityY(0) // Drive left with negative X (left)
            .withRotationalRate(turnSpeed) // Drive counterclockwise with negative X (left)
        );
    }

    @Override
    public boolean isFinished() {
        double distance = LimelightHelpers.getTY("limelight-intake");
        double angleError = -Units.degreesToRadians(LimelightHelpers.getTX("limelight-intake")); // Assume you have a method to get the angle error
        double strafeError = Math.tan(angleError);

        double anglespeed = kp_Angle * angleError;
        double forwardSpeed = kP_Distance * distance;

        return Math.abs(forwardSpeed) < 0.04
            &&Math.abs(anglespeed) < 0.04; 
        // return false;
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the drivetrain when the command ends
        m_drivetrain.setControl(
        drive.withVelocityX(0) // Drive forward with negative Y (forward)
            .withVelocityY(0) // Drive left with negative X (left)
            .withRotationalRate(0)); // Drive counterclockwise with negative X (left)
    }
}