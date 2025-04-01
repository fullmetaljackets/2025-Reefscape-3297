package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.IntakeMotorOne;


/**
 *
 */
public class IntakeMotorOneRun extends Command {

    private final IntakeMotorOne s_IntakeOne;
    private double m_IntakeSpeed;
 

    public IntakeMotorOneRun(double IntakeSpeed, IntakeMotorOne subsystem) {
        m_IntakeSpeed = IntakeSpeed;

        s_IntakeOne = subsystem;
        addRequirements(s_IntakeOne);

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        s_IntakeOne.IntakeMotorOneRun(m_IntakeSpeed);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        s_IntakeOne.IntakeMotorOneRun(0);
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