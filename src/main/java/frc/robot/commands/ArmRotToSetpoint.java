// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmRot;

public class ArmRotToSetpoint extends Command {
  private final ArmRot m_ArmRot;
  private double m_setpoint;
  private double m_tolernace;
  /** Creates a new Arm. */
  public ArmRotToSetpoint(double setpoint,double tolerance, ArmRot subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_ArmRot = subsystem;
    m_setpoint = setpoint;
    m_tolernace = tolerance;
    addRequirements(m_ArmRot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_ArmRot.setMy_ArmRot(m_setpoint);
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
    return m_ArmRot.atTargetPosition(m_setpoint, m_tolernace);
  }
}
