package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.WristRot;


/**
 *
 */
public class ResetWristRot extends Command {

    private final WristRot s_WristRot;
 

    public ResetWristRot( WristRot subsystem) {

        s_WristRot = subsystem;
        addRequirements(s_WristRot);

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        s_WristRot.resetWristRot(0);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
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