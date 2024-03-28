// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.GlobalVariables;
import frc.robot.Constants.LedConstants;
import frc.robot.GlobalVariables.RobotState;
import frc.robot.subsystems.SUB_Led;

public class CMD_LedDefault extends Command {
  final SUB_Led m_led;
  final GlobalVariables m_variables;
  public CMD_LedDefault(SUB_Led p_led, GlobalVariables p_variables) {
    m_led = p_led;
    m_variables = p_variables;
    addRequirements(m_led);
  }

  @Override
  public void execute() {
    if(m_variables.isRobotState(RobotState.ReadyToIntake)){
      m_led.setPattern(LedConstants.kChasePattern);
      m_led.setLedColors(LedConstants.kBrightOrange, LedConstants.kBrightOrange);
      m_led.setPatternDuration(2);
    }
    if(m_variables.isRobotState(RobotState.Home)
        || m_variables.isRobotState(RobotState.Stow)
        || m_variables.isRobotState(RobotState.TransitioningToHome)
    ){
      m_led.setPattern(LedConstants.kSolidColorPattern);
      m_led.setLedColors(LedConstants.kBrightGreen, LedConstants.kBrightGreen);
    }
  }

}
