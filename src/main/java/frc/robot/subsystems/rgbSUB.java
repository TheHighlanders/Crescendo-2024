package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class rgbSUB extends SubsystemBase {
    private AddressableLED m_led;
    private AddressableLEDBuffer m_ledBuffer;
    private int m_rainbowFirstPixelHue;

    public rgbSUB() {
        m_led = new AddressableLED(7);
        m_ledBuffer = new AddressableLEDBuffer(12);
        m_led.setLength(m_ledBuffer.getLength());
//        m_led.setData(m_ledBuffer);
        m_led.start();

    }

    public void rainbow() {
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;
            m_ledBuffer.setHSV(i, hue, 255, 128);
        }
        m_rainbowFirstPixelHue += 3; // Increment by 3 for visible change
        m_rainbowFirstPixelHue %= 180;
        m_led.setData(m_ledBuffer);
        DriverStation.reportWarning("it work", false);
    }




    @Override
    public void periodic() {
      // This method will be called once per scheduler run

    }
  }

  


  
