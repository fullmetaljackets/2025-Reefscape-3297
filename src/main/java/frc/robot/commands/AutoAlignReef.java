package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.LimelightHelpers;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Limelight;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutoAlignReef extends Command{
    
    private final Limelight s_Limelight;
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric();
    public static double LLRange;
    public static double LLStrafe;

    public AutoAlignReef(Limelight limelight){
        s_Limelight = limelight;
        addRequirements(limelight);

    }

    public void initialize(){
    }

    public void execute(){
        LLRange = s_Limelight.limelight_range_proportional();
        LLStrafe = s_Limelight.limelight_strafe_proportional();
        SmartDashboard.putNumber("LLRange", LLRange);
        SmartDashboard.putNumber("LLStrafe", LLStrafe);

            drivetrain.applyRequest(() -> drive
            .withVelocityX(LLRange)
            .withVelocityY(LLStrafe)
            .withRotationalRate(0)
            ).withInterruptBehavior(InterruptionBehavior.kCancelSelf);
        
    }
}

