package frc.robot.commands.grouped;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.Drive;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeMotorOne;
import frc.robot.subsystems.IntakeMotorTwo;
import frc.robot.subsystems.Limelight;

public class AutoDriveForward extends ParallelCommandGroup{

    public AutoDriveForward(Limelight s_Limelight, CommandSwerveDrivetrain s_CommandSwerveDrivetrain, IntakeMotorOne s_IntakeMotorOne, IntakeMotorTwo s_IntakeMotorTwo){

        addCommands(
            // new WaitCommand(5),
            // new AutoStrafeToCoral(s_CommandSwerveDrivetrain, s_Limelight),
            new Drive(s_Limelight, s_CommandSwerveDrivetrain),
            new IntakeRun(-.14,-0.1, s_IntakeMotorOne, s_IntakeMotorTwo)
            
        );
    }
}
