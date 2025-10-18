package frc.robot.commands.grouped.Archive;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ArmExtendToSetpoint;
import frc.robot.commands.ArmRotToSetpoint;
import frc.robot.commands.WristRotToSetpoint;
import frc.robot.commands.Achived.IntakeClose;
import frc.robot.commands.Achived.IntakeOpen;
import frc.robot.subsystems.ArmExtend;
import frc.robot.subsystems.ArmRot;
import frc.robot.subsystems.IntakeJaws;
import frc.robot.subsystems.WristRot;

public class AutoBackFloorIntake extends SequentialCommandGroup{
    
    public AutoBackFloorIntake(ArmRot s_ArmRot, ArmExtend s_ArmExtend, WristRot s_WristRot, IntakeJaws s_IntakeJaws){

        addCommands(
            new IntakeClose(s_IntakeJaws),
            new ArmExtendToSetpoint(-0.8,2, s_ArmExtend),
            new WristRotToSetpoint(0,0.01, s_WristRot),
            new ArmRotToSetpoint(-0.095,0.01, s_ArmRot),
            new IntakeOpen(s_IntakeJaws),
            new ArmExtendToSetpoint(1.7,3, s_ArmExtend),
            new WristRotToSetpoint(0.14,0.01, s_WristRot)
        );
    }
}
