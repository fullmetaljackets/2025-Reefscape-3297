package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class IntakeJaws {
        private DoubleSolenoid WristSolenoid;



    public IntakeJaws() {
        WristSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 1);
        WristSolenoid.set(Value.kForward);
    }

    public void my_JawsToggle(){
        WristSolenoid.toggle();
    }

}
