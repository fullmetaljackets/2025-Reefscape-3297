package frc.robot.subsystems;


import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.controls.MotionMagicVoltage;


public class ArmRot {
    
    private TalonFX ArmRotMotor;
    private TalonFXConfiguration TalonFXConfig;
    private MotorOutputConfigs MotorOutputConfig;
    final MotionMagicVoltage m_mmReq = new MotionMagicVoltage(0);
    private int m_printCount = 0;





    public ArmRot() {
        TalonFXConfig = new TalonFXConfiguration();
        MotorOutputConfig = new MotorOutputConfigs();
        MotorOutputConfig.Inverted = InvertedValue.Clockwise_Positive;
        MotorOutputConfig.NeutralMode = NeutralModeValue.Brake;
        TalonFXConfig.withMotorOutput(MotorOutputConfig);
        ArmRotMotor = new TalonFX(0);
        ArmRotMotor.getConfigurator().apply(TalonFXConfig);


            /* Configure gear ratio */
        FeedbackConfigs fdb = TalonFXConfig.Feedback;
        fdb.SensorToMechanismRatio = 1; // 12.8 rotor rotations per mechanism rotation
        
        /* Configure Motion Magic */
        MotionMagicConfigs mm = TalonFXConfig.MotionMagic;
        mm.MotionMagicCruiseVelocity = 0; // 5 (mechanism) rotations per second cruise
        mm.MotionMagicAcceleration = 0; // Take approximately 0.5 seconds to reach max vel
        mm.MotionMagicExpo_kV = 0;
        mm.MotionMagicExpo_kA = 0;
        // Take approximately 0.1 seconds to reach max accel 
        // mm.MotionMagicJerk = 1000;
        
        Slot0Configs slot0 = TalonFXConfig.Slot0;
        slot0.kS = 0; // Add 0.25 V output to overcome static friction
        slot0.kV = 0; // A velocity target of 1 rps results in 0.12 V output
        slot0.kA = 0; // An acceleration of 1 rps/s requires 0.01 V output
        slot0.kP = 0; // A position error of 0.2 rotations results in 12 V output
        slot0.kI = 0; // No output for integrated error
        slot0.kD = 0; // A velocity error of 1 rps results in 0.5 V output
        slot0.GravityType = GravityTypeValue.Arm_Cosine;
        slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;
        // slot0.kS = 0.25; // Add 0.25 V output to overcome static friction
        // slot0.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        // slot0.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
        // slot0.kP = 60; // A position error of 0.2 rotations results in 12 V output
        // slot0.kI = 0; // No output for integrated error
        // slot0.kD = 0.5; // A velocity error of 1 rps results in 0.5 V output

        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = ArmRotMotor.getConfigurator().apply(TalonFXConfig);
            if (status.isOK()) break;
        }
        if (!status.isOK()) {
            System.out.println("Could not configure device. Error: " + status.toString());
        }
    }

 
    public void periodic() {
        if (m_printCount++ > 10) {
        m_printCount = 0;
        // System.out.println("Pos: " + TriggerMotor.getPosition());
        // System.out.println("Vel: " + TriggerMotor.getVelocity());
        // System.out.println();
        SmartDashboard.putNumber("position", ArmRotMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Velocity", ArmRotMotor.getVelocity().getValueAsDouble());
        }

    }



    public void setMy_ArmRot(double setpoint){
        ArmRotMotor.setControl(m_mmReq.withPosition(setpoint).withSlot(0));
      }
}
