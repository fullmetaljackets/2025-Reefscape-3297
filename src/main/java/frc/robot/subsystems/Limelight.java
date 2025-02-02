package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import frc.robot.LimelightHelpers;
import frc.robot.generated.TunerConstants;

public class Limelight {


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
