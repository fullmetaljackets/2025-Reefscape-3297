package frc.robot.commands.grouped;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ArmExtendToSetpoint;
import frc.robot.commands.ArmRotToSetpoint;
import frc.robot.commands.WristRotToSetpoint;
import frc.robot.subsystems.ArmExtend;
import frc.robot.subsystems.ArmRot;
import frc.robot.subsystems.WristRot;

public class ReefLV4 extends SequentialCommandGroup{
    public ReefLV4(ArmRot s_ArmRot, ArmExtend s_ArmExtend, WristRot s_WristRot){

        System.out.println("Reef LV4");
        addCommands(
            new ArmExtendToSetpoint(-0.93, s_ArmExtend),
            new WristRotToSetpoint(0, s_WristRot),
            new ArmRotToSetpoint(0.48,s_ArmRot),
            new ArmExtendToSetpoint(4.8, s_ArmExtend),
            new WristRotToSetpoint(-0.15, s_WristRot)
        );
    }

}
