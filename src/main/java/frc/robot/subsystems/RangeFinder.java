// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RangeFinder extends SubsystemBase {
  final AnalogInput rangeFinder = new AnalogInput(2);

  /** Creates a new Arm. */
  public RangeFinder() {
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * Set the current angle of the arm (0 - 180 degrees).
   *
   * @param angleDeg Desired arm angle in degrees
   */
  public double returnDistance() {
    double rangeVoltage = rangeFinder.getAverageVoltage();
    double time = (rangeVoltage * 2) / 343;

    double distance = (time * 34300) / 2;
    return distance;
  }

  public boolean checkDistance(double distance){
    if (distance <= 30){
        return false;
    }
    else {
        return true;
    }
  }
}
