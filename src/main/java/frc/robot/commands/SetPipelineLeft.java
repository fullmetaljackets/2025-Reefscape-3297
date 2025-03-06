package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;

public class SetPipelineLeft extends Command{
private final LimelightHelpers m_LimelightHelpers;
    public SetPipelineLeft(LimelightHelpers limelightHelpers){
        m_LimelightHelpers = limelightHelpers;
    }

    public void initialize(){
        // m_LimelightHelpers.setPipelineIndex("limelight", 0);
    }

    public void execute(){
        m_LimelightHelpers.setPipelineIndex("limelight", 0);
    }
}
