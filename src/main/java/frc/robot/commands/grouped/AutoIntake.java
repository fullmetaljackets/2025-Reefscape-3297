package frc.robot.commands.grouped;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Drive;
import frc.robot.commands.AutoStrafeToCoral;
import frc.robot.commands.IntakeClose;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeJaws;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.IntakeMotorOne;
import frc.robot.subsystems.IntakeMotorTwo;

public class AutoIntake extends ParallelCommandGroup{

    public AutoIntake(Limelight s_Limelight, CommandSwerveDrivetrain s_CommandSwerveDrivetrain, IntakeJaws s_IntakeJaws, IntakeMotorOne s_IntakeMotorOne, IntakeMotorTwo s_IntakeMotorTwo){

        addCommands(
            new IntakeClose(s_IntakeJaws),
            new IntakeRun(-0.5, -0.6, s_IntakeMotorOne, s_IntakeMotorTwo),
            new Drive(s_Limelight, s_CommandSwerveDrivetrain)
            // new AutoDriveForward(s_Limelight, s_CommandSwerveDrivetrain, s_IntakeMotorOne, s_IntakeMotorTwo)
        );
    }
}
