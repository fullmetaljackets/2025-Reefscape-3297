package frc.robot.commands.grouped;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.ArmExtendToSetpoint;
import frc.robot.commands.ArmRotToSetpoint;
import frc.robot.commands.IntakeClose;
import frc.robot.commands.IntakeOpen;
import frc.robot.commands.IntakeMotorOneRun;
import frc.robot.commands.IntakeMotorTwoRun;
import frc.robot.commands.WristRotToSetpoint;
import frc.robot.subsystems.ArmExtend;
import frc.robot.subsystems.ArmRot;
import frc.robot.subsystems.IntakeMotorOne;
import frc.robot.subsystems.IntakeMotorTwo;
import frc.robot.subsystems.IntakeJaws;
import frc.robot.subsystems.WristRot;

public class BallIntake extends SequentialCommandGroup{
    
    public BallIntake(IntakeMotorOne s_IntakeMotorOne, IntakeMotorTwo s_IntakeMotorTwo, IntakeJaws s_IntakeJaws){

        addCommands(
            new IntakeOpen(s_IntakeJaws),
            new IntakeRun(-0.3, -0.3, s_IntakeMotorOne, s_IntakeMotorTwo).withTimeout(1).withInterruptBehavior(InterruptionBehavior.kCancelSelf),

            new IntakeRun(-0.2, -0.2, s_IntakeMotorOne, s_IntakeMotorTwo)
        );
    }
}
