package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Intake;


/**
 *
 */
public class intake extends Command {

    private final Intake s_Intake;
    private double m_ShooterVelocity;
 

    public intake(double ShooterVelocity, Intake subsystem) {
        m_ShooterVelocity = ShooterVelocity;

        s_Intake = subsystem;

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        s_Intake.IntakeRun(m_ShooterVelocity);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        s_Intake.IntakeRun(0);
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