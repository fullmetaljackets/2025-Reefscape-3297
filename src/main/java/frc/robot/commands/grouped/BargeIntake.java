package frc.robot.commands.grouped;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.ArmExtendToSetpoint;
import frc.robot.commands.ArmRotToSetpoint;
import frc.robot.commands.WristRotToSetpoint;
import frc.robot.commands.WristRotToSetpointSlow;
import frc.robot.commands.Achived.IntakeClose;
import frc.robot.subsystems.ArmExtend;
import frc.robot.subsystems.ArmRot;
import frc.robot.subsystems.IntakeJaws;
import frc.robot.subsystems.IntakeMotorOne;
import frc.robot.subsystems.IntakeMotorTwo;
import frc.robot.subsystems.WristRot;

public class BargeIntake extends ParallelCommandGroup{
    public BargeIntake(ArmRot s_ArmRot, ArmExtend s_ArmExtend, WristRot s_WristRot,IntakeJaws s_IntakeJaws, IntakeMotorOne s_IntakeMotorOne, IntakeMotorTwo s_IntakeMotorTwo){

        addCommands(
            new Barge(s_ArmRot, s_ArmExtend, s_WristRot, s_IntakeJaws),
            new IntakeRun(-.2, -.2, s_IntakeMotorOne, s_IntakeMotorTwo)
        );
    }

}
