// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.WristRot;

public class WristRotToSetpoint extends Command {
  private final WristRot m_Wrist;
  private double m_setpoint;
  /** Creates a new Arm. */
  public WristRotToSetpoint(double setpoint, WristRot subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_Wrist = subsystem;
    m_setpoint = setpoint;
    addRequirements(m_Wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Wrist.setMy_WristRot(m_setpoint);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_Wrist.atTargetPosition(m_setpoint, 0.01);
  }
}
