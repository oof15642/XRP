// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Arm;
import edu.wpi.first.wpilibj2.command.Command;

public class MoveArm extends Command{
  private Arm m_arm;
  double angle;

  public MoveArm(Arm m_arm, double angle) {
    this.m_arm = m_arm;
    this.angle = angle;
  }

  @Override
  public void execute() {
    m_arm.setAngle(angle);
  }

}
