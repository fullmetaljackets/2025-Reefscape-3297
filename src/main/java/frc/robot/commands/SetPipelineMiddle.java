package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;

public class SetPipelineMiddle extends Command{
private final LimelightHelpers m_LimelightHelpers;
    public SetPipelineMiddle(LimelightHelpers limelightHelpers){
        m_LimelightHelpers = limelightHelpers;
    }
    public void execute(){
        LimelightHelpers.setPipelineIndex("limelight-sone", 2);
    }
}
