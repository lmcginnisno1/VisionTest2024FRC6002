// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.GlobalVariables;
import frc.robot.subsystems.SUB_Shooter;
import frc.robot.subsystems.GlobalVariables.ScoringMode;

public class CMD_RevShooter extends Command {
  final SUB_Shooter m_shooter;
  final GlobalVariables m_variables;
  public CMD_RevShooter(SUB_Shooter p_shooter, GlobalVariables p_variables) {
    m_shooter = p_shooter;
    m_variables = p_variables;
  }

  @Override
  public void initialize() {
    if(m_variables.isScoringMode(ScoringMode.SPEAKER)){
      if(m_variables.getDistanceToTarget() <= 5){
        m_shooter.disable();
        m_shooter.setShooterVoltage(12);
      }else{
        m_shooter.enable();
        m_shooter.setSetpoint(ShooterConstants.kShooterInterpolator.getInterpolatedValue(m_variables.getDistanceToTarget() * 12));//convert to inces
      }
    }else if(m_variables.isScoringMode(ScoringMode.AMP)){
      m_shooter.setSetpoint(ShooterConstants.kShooterAmp);
    }
    
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
