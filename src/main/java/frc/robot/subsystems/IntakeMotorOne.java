package frc.robot.subsystems;


import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeMotorOne extends SubsystemBase{
    private TalonFX IntakeMotor1;
    // private TalonFX IntakeMotor2;
    private TalonFXConfiguration TalonFXConfig;
    private MotorOutputConfigs MotorOutputConfig;



    // public SparkMax Intake1Motor = new SparkMax(1, MotorType.kBrushless);
    // public SparkMax Intake2Motor = new SparkMax(2, MotorType.kBrushless);
    // SparkMaxConfig config = new SparkMaxConfig();


    public IntakeMotorOne() {
        TalonFXConfig = new TalonFXConfiguration();
        MotorOutputConfig = new MotorOutputConfigs();
        MotorOutputConfig.Inverted = InvertedValue.Clockwise_Positive;
        MotorOutputConfig.NeutralMode = NeutralModeValue.Brake;
        TalonFXConfig.withMotorOutput(MotorOutputConfig);

        IntakeMotor1 = new TalonFX(21);
        IntakeMotor1.getConfigurator().apply(TalonFXConfig);

        // IntakeMotor2 = new TalonFX(22);
        // IntakeMotor2.getConfigurator().apply(TalonFXConfig);


        // config.inverted(false);
        // config.idleMode(IdleMode.kBrake);
        // Intake1Motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        // Intake2Motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        
    }

    public void IntakeMotorOneRun(double setpoint){
        IntakeMotor1.set(-setpoint);
        // IntakeMotor2.set(setpoint);
        
    }
}
