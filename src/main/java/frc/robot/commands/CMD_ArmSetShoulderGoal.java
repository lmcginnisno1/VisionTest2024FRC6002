// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SUB_Arm;

public class CMD_ArmSetShoulderGoal extends Command {
  final SUB_Arm m_arm;
  double m_goal;
  public CMD_ArmSetShoulderGoal(SUB_Arm p_Arm, double p_goal) {
    m_arm = p_Arm;
    m_goal = p_goal;
  }

  @Override
  public void initialize() {
    m_arm.setShoulderGoal(m_goal);
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
