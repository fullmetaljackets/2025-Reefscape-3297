package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.ArmRot;
import frc.robot.subsystems.WristRot;


/**
 *
 */
public class WristRotRun extends Command {

    private final WristRot s_WristRot ;
    private double Speed;
 

    public WristRotRun(double ArmRotSpeed, WristRot subsystem) {
        Speed = ArmRotSpeed;

        s_WristRot = subsystem;
        addRequirements(subsystem);

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        s_WristRot.runMy_WristRot(Speed);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        s_WristRot.runMy_WristRot(0);
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