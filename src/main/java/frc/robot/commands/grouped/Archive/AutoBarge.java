package frc.robot.commands.grouped.Archive;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ArmExtendToSetpoint;
import frc.robot.commands.ArmRotToSetpoint;
import frc.robot.commands.WristRotToSetpoint;
import frc.robot.commands.WristRotToSetpointSlow;
import frc.robot.commands.Achived.IntakeClose;
import frc.robot.commands.grouped.BargeIntake;
import frc.robot.commands.grouped.IntakeRun;
import frc.robot.subsystems.ArmExtend;
import frc.robot.subsystems.ArmRot;
import frc.robot.subsystems.IntakeJaws;
import frc.robot.subsystems.IntakeMotorOne;
import frc.robot.subsystems.IntakeMotorTwo;
import frc.robot.subsystems.WristRot;

public class AutoBarge extends SequentialCommandGroup{
    public AutoBarge(ArmRot s_ArmRot, ArmExtend s_ArmExtend, WristRot s_WristRot,IntakeJaws s_IntakeJaws, IntakeMotorOne s_IntakeMotorOne, IntakeMotorTwo s_IntakeMotorTwo){

        addCommands(
            new IntakeRun(-.25, -.25, s_IntakeMotorOne, s_IntakeMotorTwo).withTimeout(0.5),
            new BargeIntake(s_ArmRot, s_ArmExtend, s_WristRot, s_IntakeJaws, s_IntakeMotorOne, s_IntakeMotorTwo)
        );
    }

}
