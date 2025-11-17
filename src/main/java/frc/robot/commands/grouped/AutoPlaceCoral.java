package frc.robot.commands.grouped;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
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

public class AutoPlaceCoral extends SequentialCommandGroup{

    public AutoPlaceCoral( WristRot s_WristRot, IntakeMotorOne s_IntakeMotorOne, IntakeMotorTwo s_IntakeMotorTwo){

        addCommands(
            new WristRotToSetpoint(0.1, 0.01, s_WristRot),
            new IntakeRun(.18, .18, s_IntakeMotorOne, s_IntakeMotorTwo).withTimeout(0.5)
        );
    }
}
