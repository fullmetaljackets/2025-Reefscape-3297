package frc.robot.commands.grouped;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ArmExtendToSetpoint;
import frc.robot.commands.ArmRotToSetpoint;
import frc.robot.commands.WristRotToSetpoint;
import frc.robot.commands.Achived.IntakeClose;
import frc.robot.commands.Achived.IntakeToggle;
import frc.robot.subsystems.ArmExtend;
import frc.robot.subsystems.ArmRot;
import frc.robot.subsystems.IntakeJaws;
import frc.robot.subsystems.WristRot;
import frc.robot.subsystems.IntakeMotorOne;
import frc.robot.subsystems.IntakeMotorTwo;

public class AutoReefLV4Over extends SequentialCommandGroup{

    public AutoReefLV4Over(ArmRot s_ArmRot, ArmExtend s_ArmExtend, WristRot s_WristRot, IntakeJaws s_IntakeJaws, IntakeMotorOne s_IntakeMotorOne, IntakeMotorTwo s_IntakeMotorTwo){

        addCommands(
            // new ArmExtendToSetpoint(-0.6,2,  s_ArmExtend),
            // new WristRotToSetpoint(-0.05,0.01, s_WristRot),
            // new ArmRotToSetpoint(-0.52,0.1, s_ArmRot),
            // new ArmExtendToSetpoint(5,1, s_ArmExtend)
            // new WristRotToSetpoint(0.1, 0.01, s_WristRot)
            // new IntakeRun(-0.04, -0.04, s_IntakeMotorOne, s_IntakeMotorTwo),
            new ReefLV4Over(s_ArmRot, s_ArmExtend, s_WristRot, s_IntakeJaws)
            // new WristRotToSetpoint(0.1, 0.01, s_WristRot)
        );
    }
}
