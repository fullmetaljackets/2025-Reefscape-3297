package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeJaws extends SubsystemBase{
        private DoubleSolenoid WristSolenoid;



    public IntakeJaws() {
        WristSolenoid = new DoubleSolenoid(10,PneumaticsModuleType.REVPH, 8, 9);
        WristSolenoid.set(Value.kReverse);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation

    }

    public void my_JawsClose(){
        WristSolenoid.set(Value.kReverse);
    }

    public void my_JawsOpen(){
        WristSolenoid.set(Value.kForward);
    }

    public void my_JawsToggle(){
        WristSolenoid.toggle();
    }

}
