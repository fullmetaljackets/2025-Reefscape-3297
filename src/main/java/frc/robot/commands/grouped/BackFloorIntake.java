package frc.robot.commands.grouped;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ArmExtendToSetpoint;
import frc.robot.commands.ArmRotToSetpoint;
import frc.robot.commands.IntakeClose;
import frc.robot.commands.WristRotToSetpoint;
import frc.robot.subsystems.ArmExtend;
import frc.robot.subsystems.ArmRot;
import frc.robot.subsystems.IntakeJaws;
import frc.robot.subsystems.WristRot;

public class BackFloorIntake extends SequentialCommandGroup{
    
    public BackFloorIntake(ArmRot s_ArmRot, ArmExtend s_ArmExtend, WristRot s_WristRot, IntakeJaws s_IntakeJaws){

        addCommands(
            new IntakeClose(s_IntakeJaws),
            new WristRotToSetpoint(0, s_WristRot),
            new ArmExtendToSetpoint(-0.93, s_ArmExtend),
            new ArmRotToSetpoint(-0.09,s_ArmRot),
            new ArmExtendToSetpoint(2.2, s_ArmExtend),
            new WristRotToSetpoint(0.15, s_WristRot)
        );
    }
}
