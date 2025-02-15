package frc.robot.commands.grouped;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ArmExtendToSetpoint;
import frc.robot.commands.ArmRotToSetpoint;
import frc.robot.commands.WristRotToSetpoint;
import frc.robot.commands.WristRotToSetpointSlow;
import frc.robot.subsystems.ArmExtend;
import frc.robot.subsystems.ArmRot;
import frc.robot.subsystems.WristRot;

public class Barge extends SequentialCommandGroup{
            public Barge(ArmRot s_ArmRot, ArmExtend s_ArmExtend, WristRot s_WristRot){

        addCommands(
            new ArmExtendToSetpoint(-0.93, s_ArmExtend),
            new WristRotToSetpoint(0, s_WristRot),
            new ArmRotToSetpoint(-0.46,s_ArmRot),
            new ArmExtendToSetpoint(4.87, s_ArmExtend),
            new WristRotToSetpointSlow(0.33, s_WristRot)
        );
    }

}
