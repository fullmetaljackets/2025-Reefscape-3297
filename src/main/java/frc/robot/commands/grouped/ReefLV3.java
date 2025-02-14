package frc.robot.commands.grouped;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ArmExtendToSetpoint;
import frc.robot.commands.ArmRotToSetpoint;
import frc.robot.commands.WristRotToSetpoint;
import frc.robot.subsystems.ArmExtend;
import frc.robot.subsystems.ArmRot;
import frc.robot.subsystems.WristRot;

public class ReefLV3 extends SequentialCommandGroup{

    public ReefLV3(ArmRot s_ArmRot, ArmExtend s_ArmExtend, WristRot s_WristRot){

        addCommands(
            // new WristRotToSetpoint(-0.1, s_WristRot),
            // new ArmExtendToSetpoint(0, s_ArmExtend),
            new ArmRotToSetpoint(0.41,s_ArmRot),
            new ArmExtendToSetpoint(0, s_ArmExtend),
            new WristRotToSetpoint(0.08, s_WristRot)
        );
    }
}
