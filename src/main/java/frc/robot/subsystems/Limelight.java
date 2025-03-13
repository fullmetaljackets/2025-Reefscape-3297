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
    if (LimelightHelpers.getFiducialID("limelight-score") == 6
    || LimelightHelpers.getFiducialID("limelight-score") == 7
    || LimelightHelpers.getFiducialID("limelight-score") == 8
    || LimelightHelpers.getFiducialID("limelight-score") == 9
    || LimelightHelpers.getFiducialID("limelight-score") == 10
    || LimelightHelpers.getFiducialID("limelight-score") == 11
    || LimelightHelpers.getFiducialID("limelight-score") == 17
    || LimelightHelpers.getFiducialID("limelight-score") == 18
    || LimelightHelpers.getFiducialID("limelight-score") == 19
    || LimelightHelpers.getFiducialID("limelight-score") == 20
    || LimelightHelpers.getFiducialID("limelight-score") == 21
    || LimelightHelpers.getFiducialID("limelight-score") == 22){
      Rotation2d angleToGoal = Rotation2d.fromDegrees(TunerConstants.IntakeLLMountAngle)
      .plus(Rotation2d.fromDegrees(LimelightHelpers.getTY("limelight-score")));

      double distance = (TunerConstants.ApriltagHeight - TunerConstants.IntakeLimelightHight) / angleToGoal.getTan();

      return distance;
    } else {
      return 0;
    }
  }
  public double getDistanceToCoral(){
    Rotation2d angleToGoal = Rotation2d.fromDegrees(TunerConstants.IntakeLLMountAngle)
    .plus(Rotation2d.fromDegrees(LimelightHelpers.getTY("intake limelight")));
  
    double distance =(TunerConstants.ApriltagHeight - TunerConstants.IntakeLimelightHight) / angleToGoal.getTan();

  return distance;
  }
}
