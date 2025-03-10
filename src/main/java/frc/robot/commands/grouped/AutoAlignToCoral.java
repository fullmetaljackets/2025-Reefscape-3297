package frc.robot.commands.grouped;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AutoDriveToCoral;
import frc.robot.commands.AutoStrafeToCoral;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AutoAlignToCoral extends SequentialCommandGroup{

    public AutoAlignToCoral(Limelight s_Limelight, CommandSwerveDrivetrain s_CommandSwerveDrivetrain){

        addCommands(
            // new WaitCommand(5),
            new AutoDriveToCoral(s_CommandSwerveDrivetrain, s_Limelight),
            new AutoStrafeToCoral(s_CommandSwerveDrivetrain, s_Limelight)

        );
    }
}
