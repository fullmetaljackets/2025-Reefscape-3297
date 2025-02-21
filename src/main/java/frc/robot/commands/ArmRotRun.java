package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.ArmRot;


/**
 *
 */
public class ArmRotRun extends Command {

    private final ArmRot s_ArmRot ;
    private double Speed;
 

    public ArmRotRun(double ArmRotSpeed, ArmRot subsystem) {
        Speed = ArmRotSpeed;

        s_ArmRot = subsystem;
        addRequirements(subsystem);

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        s_ArmRot.runMy_ArmRot(Speed);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        s_ArmRot.runMy_ArmRot(0);
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