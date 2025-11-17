package frc.robot.commands.grouped;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ArmExtendToSetpoint;
import frc.robot.commands.ArmRotToSetpoint;
import frc.robot.commands.WristRotToSetpoint;
import frc.robot.commands.Achived.IntakeClose;
import frc.robot.subsystems.ArmExtend;
import frc.robot.subsystems.ArmRot;
import frc.robot.subsystems.IntakeJaws;
import frc.robot.subsystems.WristRot;

public class Stash extends SequentialCommandGroup{

    public Stash(ArmRot s_ArmRot, ArmExtend s_ArmExtend, WristRot s_WristRot){

        addCommands(
            new ArmExtendToSetpoint(-0.8,2, s_ArmExtend),
            new WristRotToSetpoint(-0.05,0.01, s_WristRot),
            new ArmRotToSetpoint(0,0.01,s_ArmRot),
            new ArmExtendToSetpoint(-0.14, 0.1, s_ArmExtend)
        );
    }
}
