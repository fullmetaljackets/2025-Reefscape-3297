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
  
  public double limelight_range_proportional(){    
    // kP (constant of proportionality)
    // this is a hand-tuned number that determines the aggressiveness of our proportional control loop
    // if it is too high, the robot will oscillate.
    // if it is too low, the robot will never reach its target
    // if the robot never turns in the correct direction, kP should be inverted.
    double kP = .1;

    // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the rightmost edge of 
    // your limelight 3 feed, tx should return roughly 31 degrees.
    double targetingForwardSpeed = LimelightHelpers.getTY("limelight") * kP;

    // convert to radians per second for our drive method
    targetingForwardSpeed *= TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);

    //invert since tx is positive when the target is to the right of the crosshair
    targetingForwardSpeed *= -1.0;

    return targetingForwardSpeed;
  }

  public double limelight_strafe_proportional()
  {
    double kP = .1;
    double targetingStrafeSpeed = LimelightHelpers.getTX("limelight") * kP;
    targetingStrafeSpeed *= TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    targetingStrafeSpeed *= -1.0;
    return targetingStrafeSpeed;
  }

}
