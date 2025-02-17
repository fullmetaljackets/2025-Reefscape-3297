package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

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
    private final CommandSwerveDrivetrain drivetrain;
    private final Limelight limelight;
    private final double kP = 0.1; // Proportional control constant
    private final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric();

    public AutoAlignToAprilTag(CommandSwerveDrivetrain drivetrain, Limelight limelight) {
        this.drivetrain = drivetrain;
        this.limelight = limelight;
        addRequirements( limelight);
    }

    @Override
    public void initialize() {
        // Initialization code if needed
    }

    @Override
    public void execute() {
        LimelightHelpers.setPipelineIndex("limelight", 0);
        
        double distance = limelight.getDistanceToReef();
        double angleError = -Units.degreesToRadians(LimelightHelpers.getTX("limelight")); // Assume you have a method to get the angle error
        double strafeError = distance * Math.tan(angleError);
        
        // Proportional control for distance and angle
        double forwardSpeed = kP * distance;
        double turnSpeed = kP * angleError;
        double strafeSpeed = kP * strafeError;

        SmartDashboard.putNumber("forward speed", forwardSpeed);
        SmartDashboard.putNumber("turn speed", turnSpeed);
        SmartDashboard.putNumber("strafe speed", strafeSpeed);

        // Drive the robot
        // drivetrain.arcadeDrive(forwardSpeed, turnSpeed);

        drivetrain.applyRequest(() ->
        drive.withVelocityX(forwardSpeed) // Drive forward with negative Y (forward)
            .withVelocityY(strafeSpeed) // Drive left with negative X (left)
            .withRotationalRate(turnSpeed) // Drive counterclockwise with negative X (left)
        );
    }

    @Override
    public boolean isFinished() {
        double distance = limelight.getDistanceToReef();
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