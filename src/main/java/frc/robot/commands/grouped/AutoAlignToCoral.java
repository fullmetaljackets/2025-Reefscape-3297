package frc.robot.commands.grouped;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoStrafeToCoral;
import frc.robot.commands.Drive;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeJaws;
import frc.robot.subsystems.IntakeMotorOne;
import frc.robot.subsystems.IntakeMotorTwo;
import frc.robot.subsystems.Limelight;

public class AutoAlignToCoral extends SequentialCommandGroup{

    public AutoAlignToCoral(Limelight s_Limelight, CommandSwerveDrivetrain s_CommandSwerveDrivetrain, IntakeJaws s_IntakeJaws, IntakeMotorOne s_IntakeMotorOne, IntakeMotorTwo s_IntakeMotorTwo){

        addCommands(
            // new WaitCommand(5),
            // new AutoStrafeToCoral(s_CommandSwerveDrivetrain, s_Limelight),
            new AutoStrafeToCoral(s_CommandSwerveDrivetrain, s_Limelight),
            // new Drive(s_Limelight, s_CommandSwerveDrivetrain).withTimeout(1),
            // new AutoIntake(s_Limelight, s_CommandSwerveDrivetrain, s_IntakeJaws, s_IntakeMotorOne,s_IntakeMotorTwo).withTimeout(1.2)
            // new WaitCommand(.3), 
            new AutoIntake(s_Limelight, s_CommandSwerveDrivetrain, s_IntakeJaws, s_IntakeMotorOne, s_IntakeMotorTwo).withTimeout(1.8)
            // ,new WaitCommand(3)
        );
    }
}
