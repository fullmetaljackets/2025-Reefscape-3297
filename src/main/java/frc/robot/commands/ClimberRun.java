package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Climber;


/**
 *
 */
public class ClimberRun extends Command {

    private final Climber s_Climber ;
    private double Speed;
 

    public ClimberRun(double ClimberSpeed, Climber subsystem) {
        Speed = ClimberSpeed;

        s_Climber = subsystem;
        addRequirements(s_Climber);

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        s_Climber.runMy_ArmRot(Speed);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        s_Climber.runMy_ArmRot(0);
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