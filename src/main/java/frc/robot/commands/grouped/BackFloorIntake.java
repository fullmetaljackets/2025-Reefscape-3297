package frc.robot.commands.grouped;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ArmExtendToSetpoint;
import frc.robot.commands.ArmRotToSetpoint;
import frc.robot.commands.WristRotToSetpoint;
import frc.robot.subsystems.ArmExtend;
import frc.robot.subsystems.ArmRot;
import frc.robot.subsystems.WristRot;

public class BackFloorIntake extends SequentialCommandGroup{
    
    public BackFloorIntake(ArmRot s_ArmRot, ArmExtend s_ArmExtend, WristRot s_WristRot){

        addCommands(
            //shooter motor 1&2 out
            // new WristRotToSetpoint(-.1, s_WristRot)
                // .andThen(new ArmExtendToSetpoint(0,s_ArmExtend))
                //start swing
                new ArmRotToSetpoint(-0.1,s_ArmRot)
                .andThen(new ArmExtendToSetpoint(2.42, s_ArmExtend))
                .andThen(new WristRotToSetpoint(0.18, s_WristRot))
            // new ArmExtendToSetpoint(0, s_ArmExtend),
            // new WristRotToSetpoint(0.04, s_WristRot)
        );
    }
}
