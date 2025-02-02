package frc.robot.subsystems;


import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DriverStation;

public class Intake {
    // private SparkMax Intake1Motor;
    // private SparkMax Intake2Motor;
    public SparkMax Intake1Motor = new SparkMax(0, MotorType.kBrushless);
    public SparkMax Intake2Motor = new SparkMax(0, MotorType.kBrushless);
    SparkMaxConfig config = new SparkMaxConfig();


    public Intake() {
        config.inverted(false);
        config.idleMode(IdleMode.kCoast);
        Intake1Motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        Intake2Motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        
    }

    public void IntakeRun(double setpoint){
        Intake1Motor.set(setpoint);
        Intake2Motor.set(setpoint);
    }
}
