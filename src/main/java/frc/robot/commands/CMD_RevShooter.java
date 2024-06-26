// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.GlobalVariables;
import frc.robot.Constants.ShooterConstants;
import frc.robot.GlobalVariables.ScoringMode;
import frc.robot.subsystems.SUB_Shooter;

public class CMD_RevShooter extends Command {
  final SUB_Shooter m_shooter;
  final GlobalVariables m_variables;
  final Timer m_revTimer = new Timer();
  public CMD_RevShooter(SUB_Shooter p_shooter, GlobalVariables p_variables) {
    m_shooter = p_shooter;
    m_variables = p_variables;
  }

  @Override
  public void initialize() {
    m_revTimer.stop();
    m_revTimer.reset();

    if(m_variables.isScoringMode(ScoringMode.SPEAKER)){
      if(m_variables.getDistanceToTarget() <= 30){
        m_shooter.setSetpoint(ShooterConstants.kShooterSubwoofer);
        m_revTimer.start();
      }else{
        m_shooter.setSetpoint(ShooterConstants.kShooterLongshotSpeed);
      }
    }else if(m_variables.isScoringMode(ScoringMode.AMP)){
      m_shooter.setSetpoint(ShooterConstants.kShooterAmp);
    }else{
      return;
    }
    m_shooter.enable();
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
