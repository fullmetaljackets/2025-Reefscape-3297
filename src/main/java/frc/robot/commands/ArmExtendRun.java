package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmExtend;
import frc.robot.subsystems.ArmRot;


/**
 *
 */
public class ArmExtendRun extends Command {

    private final ArmExtend s_ArmExtend ;
    private double Speed;
 

    public ArmExtendRun(double ArmRotSpeed, ArmExtend subsystem) {
        Speed = ArmRotSpeed;

        s_ArmExtend = subsystem;
        addRequirements(subsystem);

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        s_ArmExtend.RunMy_ArmExtend(Speed);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        s_ArmExtend.RunMy_ArmExtend(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public boolean runsWhenDisabled() {
        return false;
    }
}