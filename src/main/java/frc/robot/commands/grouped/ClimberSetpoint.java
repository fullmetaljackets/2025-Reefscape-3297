package frc.robot.commands.grouped;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.ClimberToSetpoint;
import frc.robot.commands.grouped.Climb;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.ArmRot;
import frc.robot.subsystems.ArmExtend;
import frc.robot.subsystems.WristRot;


public class ClimberSetpoint extends ParallelCommandGroup{

    public ClimberSetpoint(Climber s_Climber, ArmRot s_ArmRot, ArmExtend s_ArmExtend, WristRot s_WristRot){

        addCommands(
            new ClimberToSetpoint(0.39, 0,  s_Climber),
            new Climb(s_ArmRot, s_ArmExtend, s_WristRot)
            // new AutoDriveForward(s_Limelight, s_CommandSwerveDrivetrain, s_IntakeMotorOne, s_IntakeMotorTwo)
        );
    }
}
