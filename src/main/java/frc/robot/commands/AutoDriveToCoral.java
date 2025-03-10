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
    private final double kP_Distance = 0.034; // Proportional control constant
    private final double kp_Strafe = 1.2;
    private final double DistanceOffset = 0;
    private final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric();
    // double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0.0);

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
        // double angleError = -Units.degreesToRadians(LimelightHelpers.getTX("limelight-intake")); // Assume you have a method to get the angle error
        // double strafeError = Math.tan(angleError);


        double forwardSpeed = kP_Distance * distance;
        // double strafeSpeed = kp_Strafe * strafeError;

        SmartDashboard.putNumber("intake LL distance", distance);
        SmartDashboard.putNumber("intake LL forward speed", forwardSpeed);

        // SmartDashboard.putNumber("intake LL strafe error", strafeError);
        // SmartDashboard.putNumber("intake LL strafe speed", strafeSpeed);

        m_drivetrain.setControl(
        m_alignRequest.withVelocityX(-forwardSpeed) // Drive forward with negative Y (forward)
            .withVelocityY(0 ) // Drive left with negative X (left)
            .withRotationalRate(0) // Drive counterclockwise with negative X (left)
        );
    }

    @Override
    public boolean isFinished() {
        double distance = LimelightHelpers.getTY("limelight-intake");
        // double angleError = -Units.degreesToRadians(LimelightHelpers.getTX("limelight-intake")); // Assume you have a method to get the angle error
        // double strafeError = Math.tan(angleError);

        // double strafeSpeed = kp_Strafe * strafeError;
        double forwardSpeed = kP_Distance * distance;

        return Math.abs(forwardSpeed) < 0.04;
            // &&Math.abs(strafeSpeed) < 0.04; 
        // return false;
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the drivetrain when the command ends
        // m_drivetrain.setControl(
        // drive.withVelocityX(0) // Drive forward with negative Y (forward)
        //     .withVelocityY(0) // Drive left with negative X (left)
        //     .withRotationalRate(0)); // Drive counterclockwise with negative X (left)
        // );
    }
}