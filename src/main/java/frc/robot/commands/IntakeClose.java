package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.subsystems.IntakeJaws;

/**
 *
 */
public class IntakeClose extends InstantCommand {
    private final IntakeJaws m_IntakeJaws;

    public IntakeClose(IntakeJaws subsystem) {
        m_IntakeJaws = subsystem;
    }

    // Called once when this command runs
    @Override
    public void initialize() {
        m_IntakeJaws.my_JawsClose();
    }

    @Override
    public boolean runsWhenDisabled() {
        return false;
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
