package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;

public class SetPipelineRight extends Command{
private final LimelightHelpers m_LimelightHelpers;
    public SetPipelineRight(LimelightHelpers limelightHelpers){
        m_LimelightHelpers = limelightHelpers;
    }
    public void execute(){
        LimelightHelpers.setPipelineIndex("limelight", 0);
    }
}
