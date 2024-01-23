// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.TreeMap;
import edu.wpi.first.math.InterpolatingMatrixTreeMap;
import edu.wpi.first.math.Num;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class InterpolatingMatrixShotTreeMap extends SubsystemBase {

  InterpolatingMatrixTreeMap<Double, Num, Num> interpolatingMap =
      new InterpolatingMatrixTreeMap<>();

  public InterpolatingMatrixShotTreeMap() {
    super();
  }

  private TreeMap<Double, Double> createTreeMap(double key, double value1, double value2) {
    TreeMap<Double, Double> treeMap = new TreeMap<>();
    treeMap.put(key, value1);
    // You can add more values to the TreeMap if needed
    return treeMap;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
