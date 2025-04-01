package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.generated.TunerConstants;

public class Limelight extends SubsystemBase{

  public double getDistanceToReef(){
    if (LimelightHelpers.getFiducialID("limelight-sone") == 6
    || LimelightHelpers.getFiducialID("limelight-sone") == 7
    || LimelightHelpers.getFiducialID("limelight-sone") == 8
    || LimelightHelpers.getFiducialID("limelight-sone") == 9
    || LimelightHelpers.getFiducialID("limelight-sone") == 10
    || LimelightHelpers.getFiducialID("limelight-sone") == 11
    || LimelightHelpers.getFiducialID("limelight-sone") == 17
    || LimelightHelpers.getFiducialID("limelight-sone") == 18
    || LimelightHelpers.getFiducialID("limelight-sone") == 19
    || LimelightHelpers.getFiducialID("limelight-sone") == 20
    || LimelightHelpers.getFiducialID("limelight-sone") == 21
    || LimelightHelpers.getFiducialID("limelight-sone") == 22){
      Rotation2d angleToGoal = Rotation2d.fromDegrees(TunerConstants.IntakeLLMountAngle)
      .plus(Rotation2d.fromDegrees(LimelightHelpers.getTY("limelight-sone")));

      double distance = (TunerConstants.ApriltagHeight - TunerConstants.IntakeLimelightHight) / angleToGoal.getTan();

      return distance;
    } else {
      if (LimelightHelpers.getFiducialID("limelight-stwo") == 6
      || LimelightHelpers.getFiducialID("limelight-stwo") == 7
      || LimelightHelpers.getFiducialID("limelight-stwo") == 8
      || LimelightHelpers.getFiducialID("limelight-stwo") == 9
      || LimelightHelpers.getFiducialID("limelight-stwo") == 10
      || LimelightHelpers.getFiducialID("limelight-stwo") == 11
      || LimelightHelpers.getFiducialID("limelight-stwo") == 17
      || LimelightHelpers.getFiducialID("limelight-stwo") == 18
      || LimelightHelpers.getFiducialID("limelight-stwo") == 19
      || LimelightHelpers.getFiducialID("limelight-stwo") == 20
      || LimelightHelpers.getFiducialID("limelight-stwo") == 21
      || LimelightHelpers.getFiducialID("limelight-stwo") == 22){
        Rotation2d angleToGoal = Rotation2d.fromDegrees(TunerConstants.IntakeLLMountAngle)
        .plus(Rotation2d.fromDegrees(LimelightHelpers.getTY("limelight-stwo")));
  
        double distance = (TunerConstants.ApriltagHeight - TunerConstants.IntakeLimelightHight) / angleToGoal.getTan();
  
        return distance;
      } else {
        return 0;
     }
  }
  }
  public double getDistanceToCoral(){
    Rotation2d angleToGoal = Rotation2d.fromDegrees(TunerConstants.IntakeLLMountAngle)
    .plus(Rotation2d.fromDegrees(LimelightHelpers.getTY("intake limelight")));
  
    double distance =(TunerConstants.ApriltagHeight - TunerConstants.IntakeLimelightHight) / angleToGoal.getTan();

  return distance;
  }
}
