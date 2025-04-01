package frc.robot.commands.grouped;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.IntakeMotorOneRun;
import frc.robot.commands.IntakeMotorTwoRun;
import frc.robot.subsystems.ArmRot;
import frc.robot.subsystems.IntakeMotorOne;
import frc.robot.subsystems.IntakeMotorTwo;

public class Score extends ParallelCommandGroup{
    public Score(double IntakeMotor1Speed, double IntakeMotor2Speed, IntakeMotorOne s_IntakeMotorOne, IntakeMotorTwo s_IntakeMotorTwo, ArmRot arm){

        // if (arm.getArmRotPosition() > -0.5 ) {
        //     IntakeMotor1Speed = IntakeMotor1Speed * -1;
        //     IntakeMotor2Speed = IntakeMotor2Speed * -1;
        // };

        addCommands(
            new IntakeMotorOneRun(IntakeMotor1Speed, s_IntakeMotorOne),
            new IntakeMotorTwoRun(IntakeMotor2Speed, s_IntakeMotorTwo)
        );
    }

}
