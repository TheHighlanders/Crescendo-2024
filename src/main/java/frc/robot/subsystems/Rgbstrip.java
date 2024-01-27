package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Rgbstrip extends SubsystemBase {
  DigitalOutput[] dios = {new DigitalOutput(4), new DigitalOutput(5), new DigitalOutput(6), new DigitalOutput(7)};
  
  public Rgbstrip() {
  }

  public void sendDiostrip(int strip){
    String binary = Integer.toBinaryString(strip);
  
    SmartDashboard.putString("Binary", binary + " " + strip);

    for (int i = binary.length()-1; i >= 0; i--) {
        dios[binary.length() - i - 1].set(binary.charAt(i) == '1');
    }
  }


  @Override
  public void periodic() {
      // This method will be called once per scheduler run

    // dios[0].set(true);
    // dios[3].set(true);
    // dios[1].set(true);
    // dios[2].set(true);
  }
}
  

  


  
