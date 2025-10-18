package frc.robot.commands.grouped;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ArmExtendToSetpoint;
import frc.robot.commands.ArmRotToSetpoint;
import frc.robot.commands.WristRotToSetpoint;
import frc.robot.commands.Achived.IntakeClose;
import frc.robot.commands.Achived.IntakeOpen;
import frc.robot.subsystems.ArmExtend;
import frc.robot.subsystems.ArmRot;
import frc.robot.subsystems.IntakeJaws;
import frc.robot.subsystems.IntakeMotorOne;
import frc.robot.subsystems.IntakeMotorTwo;
import frc.robot.subsystems.WristRot;

public class BackFloorIntake extends SequentialCommandGroup{
    
    public BackFloorIntake(ArmRot s_ArmRot, ArmExtend s_ArmExtend, WristRot s_WristRot, IntakeJaws s_IntakeJaws, IntakeMotorOne s_IntakeMotorOne, IntakeMotorTwo s_IntakeMotorTwo){

        addCommands(
            new IntakeClose(s_IntakeJaws),
            // new ArmExtendToSetpoint(-0.93,2, s_ArmExtend),
            new ArmExtendToSetpoint(-0.7,5, s_ArmExtend),
            new WristRotToSetpoint(0,0.01, s_WristRot),
            // new ArmRotToSetpoint(-0.09,0.01, s_ArmRot),
            new ArmRotToSetpoint(-0.115,0.05, s_ArmRot),
            // new ArmExtendToSetpoint(1.8,3, s_ArmExtend),
            new ArmExtendToSetpoint(0.95,3, s_ArmExtend),
            // new WristRotToSetpoint(0.15,0.01, s_WristRot)
            new WristRotToSetpoint(0.31,0.01, s_WristRot)
            // new IntakeRun(-0.6 ,-0.5, s_IntakeMotorOne, s_IntakeMotorTwo) // Adjust the speed as needed

        );
    }
}
