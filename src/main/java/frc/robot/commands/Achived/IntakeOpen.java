package frc.robot.commands.Achived;

import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.subsystems.IntakeJaws;

/**
 *
 */
public class IntakeOpen extends InstantCommand {
    private final IntakeJaws m_IntakeJaws;

    public IntakeOpen(IntakeJaws subsystem) {
        m_IntakeJaws = subsystem;
    }

    // Called once when this command runs
    @Override
    public void initialize() {
        m_IntakeJaws.my_JawsOpen();
    }

    @Override
    public boolean runsWhenDisabled() {
        return false;
    }
}
