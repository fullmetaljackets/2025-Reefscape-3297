package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.IntakeMotorTwo;


/**
 *
 */
public class IntakeMotorTwoRun extends Command {

    private final IntakeMotorTwo s_IntakeTwo;
    private double m_IntakeSpeed;
 

    public IntakeMotorTwoRun(double IntakeSpeed, IntakeMotorTwo subsystem) {
        m_IntakeSpeed = IntakeSpeed;

        s_IntakeTwo = subsystem;
        addRequirements(s_IntakeTwo);

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        s_IntakeTwo.IntakeMotorTwoRun(m_IntakeSpeed);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        s_IntakeTwo.IntakeMotorTwoRun(0);
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