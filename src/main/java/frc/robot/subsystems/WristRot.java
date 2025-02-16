package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;

public class WristRot extends SubsystemBase {

    private TalonFX WristRotMotor;
    private TalonFXConfiguration TalonFXConfig;
    private MotorOutputConfigs MotorOutputConfig;
    final MotionMagicExpoVoltage m_mmReq = new MotionMagicExpoVoltage(0);
    private int m_printCount = 0;


    public WristRot() {
        TalonFXConfig = new TalonFXConfiguration();
        MotorOutputConfig = new MotorOutputConfigs();
        MotorOutputConfig.Inverted = InvertedValue.CounterClockwise_Positive;
        MotorOutputConfig.NeutralMode = NeutralModeValue.Brake;
        TalonFXConfig.withMotorOutput(MotorOutputConfig);
        WristRotMotor = new TalonFX(3, "rio");
        WristRotMotor.getConfigurator().apply(TalonFXConfig);


            /* Configure gear ratio */
        FeedbackConfigs fdb = TalonFXConfig.Feedback;
        fdb.SensorToMechanismRatio = 125; // 125 rotor rotations per mechanism rotation
        
        /* Configure Motion Magic */
        MotionMagicConfigs mm = TalonFXConfig.MotionMagic;
        mm.MotionMagicCruiseVelocity = 0; // 5 (mechanism) rotations per second cruise
        mm.MotionMagicAcceleration = 0; // Take approximately 0.5 seconds to reach max vel
        mm.MotionMagicExpo_kV = 0.11999999731779099;
        mm.MotionMagicExpo_kA = 0.10000000149011612;
        // Take approximately 0.1 seconds to reach max accel 
        // mm.MotionMagicJerk = 1000;

        SoftwareLimitSwitchConfigs softLimit =TalonFXConfig.SoftwareLimitSwitch;
        softLimit.ForwardSoftLimitEnable = true;
        softLimit.ForwardSoftLimitThreshold = 0.6;
        softLimit.ReverseSoftLimitEnable = true;
        softLimit.ReverseSoftLimitThreshold = 0;

        
        Slot0Configs slot0 = TalonFXConfig.Slot0;
        slot0.kS = 0; // Add 0.25 V output to overcome static friction
        slot0.kV = 0; // A velocity target of 1 rps results in 0.12 V output
        slot0.kA = 0; // An acceleration of 1 rps/s requires 0.01 V output
        slot0.kG = 0.18; // voltage output to overcome gravity 
        slot0.kP = 60; // A position error of 0.2 rotations results in 12 V output
        slot0.kI = 0; // No output for integrated error
        slot0.kD = 0; // A velocity error of 1 rps results in 0.5 V output
        slot0.GravityType = GravityTypeValue.Arm_Cosine;
        slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;

        Slot1Configs slot1 = TalonFXConfig.Slot1;
        slot1.kS = 0; // Add 0.25 V output to overcome static friction
        slot1.kV = 0; // A velocity target of 1 rps results in 0.12 V output
        slot1.kA = 0; // An acceleration of 1 rps/s requires 0.01 V output
        slot1.kG = 0.18; // voltage output to overcome gravity 
        slot1.kP = 20; // A position error of 0.2 rotations results in 12 V output
        slot1.kI = 0; // No output for integrated error
        slot1.kD = 0; // A velocity error of 1 rps results in 0.5 V output
        slot1.GravityType = GravityTypeValue.Arm_Cosine;
        slot1.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;

        // slot0.kS = 0.25; // Add 0.25 V output to overcome static friction
        // slot0.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        // slot0.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
        // slot0.kP = 60; // A position error of 0.2 rotations results in 12 V output
        // slot0.kI = 0; // No output for integrated error
        // slot0.kD = 0.5; // A velocity error of 1 rps results in 0.5 V output

        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = WristRotMotor.getConfigurator().apply(TalonFXConfig);
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
        SmartDashboard.putNumber("position", WristRotMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Velocity", WristRotMotor.getVelocity().getValueAsDouble());
        }
    }

    public double getWristPosition(){
        return WristRotMotor.getPosition().getValueAsDouble();
    }
    
    public boolean atTargetPosition(double setpoint, double tolerance){
        return Math.abs(getWristPosition() - setpoint) <= tolerance;
    }

    public void setMy_WristRot(double setpoint){
    WristRotMotor.setControl(m_mmReq.withPosition(setpoint).withSlot(0));
    }
    public void setMy_WristRotSlow(double setpoint){
    WristRotMotor.setControl(m_mmReq.withPosition(setpoint).withSlot(1));
    }

}