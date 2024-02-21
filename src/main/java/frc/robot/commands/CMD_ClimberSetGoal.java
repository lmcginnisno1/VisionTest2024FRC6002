// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SUB_Climber;

public class CMD_ClimberSetGoal extends Command {
  final SUB_Climber m_Climber;
  double m_goal;
  public CMD_ClimberSetGoal(SUB_Climber p_climber, double p_goal) {
    m_Climber = p_climber;
    m_goal = p_goal;
  }

  @Override
  public void initialize() {
    m_Climber.setWinchGoal(m_goal);
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
