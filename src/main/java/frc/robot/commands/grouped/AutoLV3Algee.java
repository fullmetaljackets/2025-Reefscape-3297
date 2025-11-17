package frc.robot.commands.grouped;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AutoAlignLV3Algee;
import frc.robot.commands.Achived.AutoStrafeToCoral;
import frc.robot.commands.Achived.Drive;
import frc.robot.commands.Achived.IntakeClose;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeJaws;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.IntakeMotorOne;
import frc.robot.subsystems.IntakeMotorTwo;

public class AutoLV3Algee extends ParallelCommandGroup{

    public AutoLV3Algee(Limelight s_Limelight, CommandSwerveDrivetrain s_CommandSwerveDrivetrain, IntakeJaws s_IntakeJaws, IntakeMotorOne s_IntakeMotorOne, IntakeMotorTwo s_IntakeMotorTwo){

        addCommands(
            new IntakeRun(-0.3, -0.3, s_IntakeMotorOne, s_IntakeMotorTwo).withTimeout(1),
            new AutoAlignLV3Algee(s_CommandSwerveDrivetrain, s_Limelight).withTimeout(1.2)
        );
    }
}
