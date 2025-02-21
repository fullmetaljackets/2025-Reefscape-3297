package frc.robot.subsystems;


import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase{
    public SparkMax Intake1Motor = new SparkMax(1, MotorType.kBrushless);
    public SparkMax Intake2Motor = new SparkMax(2, MotorType.kBrushless);
    SparkMaxConfig config = new SparkMaxConfig();


    public Intake() {
        config.inverted(false);
        config.idleMode(IdleMode.kBrake);
        Intake1Motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        Intake2Motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        
    }

    public void IntakeRun(double setpoint){
        Intake1Motor.set(-setpoint);
        Intake2Motor.set(setpoint);
    }
}
