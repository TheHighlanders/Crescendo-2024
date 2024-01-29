package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Rgbstate extends SubsystemBase {
  DigitalOutput[] dios = {new DigitalOutput(0), new DigitalOutput(1), new DigitalOutput(2), new DigitalOutput(3)};
  
  public Rgbstate() {
  }

  public void sendDiostate(int numstate){
    String binary = Integer.toBinaryString(numstate);
  
    SmartDashboard.putString("Binary", binary + " " + numstate);

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
  

  


  
