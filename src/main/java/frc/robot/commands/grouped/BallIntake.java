package frc.robot.commands.grouped;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.ArmExtendToSetpoint;
import frc.robot.commands.ArmRotToSetpoint;
import frc.robot.commands.IntakeClose;
import frc.robot.commands.IntakeOpen;
import frc.robot.commands.IntakeRun;
import frc.robot.commands.WristRotToSetpoint;
import frc.robot.subsystems.ArmExtend;
import frc.robot.subsystems.ArmRot;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeJaws;
import frc.robot.subsystems.WristRot;

public class BallIntake extends SequentialCommandGroup{
    
    public BallIntake(Intake s_Intake, IntakeJaws s_IntakeJaws){

        addCommands(
            new IntakeOpen(s_IntakeJaws),
            new IntakeRun(-0.2, s_Intake).withTimeout(1),
            // new WaitCommand(1),
            new IntakeRun(-0.6, s_Intake),
            new IntakeClose(s_IntakeJaws)
        );
    }
}
