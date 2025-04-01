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

public class ReefLV1 extends SequentialCommandGroup{
        public ReefLV1(ArmRot s_ArmRot, ArmExtend s_ArmExtend, WristRot s_WristRot, IntakeJaws s_IntakeJaws){

        addCommands(
            // new IntakeClose(s_IntakeJaws),
            new ArmExtendToSetpoint(-0.73,4.5, s_ArmExtend),
            new WristRotToSetpoint(0,0.01, s_WristRot),
            new ArmRotToSetpoint(-0.18,0.1,s_ArmRot),
            new ArmExtendToSetpoint(-0.73,0.1, s_ArmExtend),
            new WristRotToSetpoint(0,0.01, s_WristRot)
        );
    }

}
