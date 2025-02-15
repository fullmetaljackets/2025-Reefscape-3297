package frc.robot.commands.grouped;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ArmExtendToSetpoint;
import frc.robot.commands.ArmRotToSetpoint;
import frc.robot.commands.WristRotToSetpoint;
import frc.robot.subsystems.ArmExtend;
import frc.robot.subsystems.ArmRot;
import frc.robot.subsystems.WristRot;

public class ReefLV2 extends SequentialCommandGroup{
    public ReefLV2(ArmRot s_ArmRot, ArmExtend s_ArmExtend, WristRot s_WristRot){

        addCommands(
            new ArmExtendToSetpoint(-0.93, s_ArmExtend),
            new WristRotToSetpoint(0, s_WristRot),
            new ArmRotToSetpoint(0.34,s_ArmRot),
            new ArmExtendToSetpoint(-0.93, s_ArmExtend),
            new WristRotToSetpoint(0, s_WristRot)
        );
    }

}
