package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
  public static final int kPort = 9;
  public static final int kLength = 77;

  public final AddressableLED m_led;
  public final AddressableLEDBuffer m_buffer;
  public final Color yellow = new Color(200, 252, 0);

  public LEDSubsystem() {
    m_led = new AddressableLED(kPort);
    m_buffer = new AddressableLEDBuffer(kLength);
    m_led.setLength(kLength);
    m_led.start();

    // Set the default command to turn the strip off, otherwise the last colors written by
    // the last command to run will continue to be displayed.
    // Note: Other default patterns could be used instead!

    // setDefaultCommand(runPattern(LEDPattern.gradient(GradientType.kContinuous,  Color.kRed, Color.kBlue)));
    setDefaultCommand(runPattern(LEDPattern.solid(yellow)));
    
    // runPattern(LEDPattern.rainbow(255, 128)).withName("rainbow");
  }

  @Override
  public void periodic() {
    // Periodically send the latest LED color data to the LED strip for it to display
    // runPattern(LEDPattern.solid(Color.kYellow)).withName("yellow");
    m_led.setData(m_buffer);
  }

  /**
   * Creates a command that runs a pattern on the entire LED strip.
   *
   * @param pattern the LED pattern to run
   */
  public Command runPattern(LEDPattern pattern) {
    return run(() -> pattern.applyTo(m_buffer));
  }

  // public void SetYellow(){
  //   // Create an LED pattern that sets the entire strip to solid red
  // LEDPattern yellow = LEDPattern.solid(Color.kYellow);

  //   // Apply the LED pattern to the data buffer
  //   yellow.applyTo(m_buffer);

  //   // Write the data to the LED strip
  //   m_led.setData(m_buffer);
  // }
}