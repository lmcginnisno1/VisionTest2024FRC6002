// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.GlobalVariables;
import frc.robot.Constants.ArmConstants;
import frc.robot.GlobalVariables.ScoringMode;
import frc.robot.subsystems.SUB_Arm;

public class CMD_ArmAim extends Command {
  final GlobalVariables m_variables;
  final SUB_Arm m_arm;
  boolean isFinished;
  boolean noWait;
  public CMD_ArmAim(SUB_Arm p_arm, GlobalVariables p_variables) {
    m_arm = p_arm;
    m_variables = p_variables;
  }

  @Override
  public void initialize() {
    isFinished = false;
    noWait = false;
    //if in AMP scoring mode
    if((m_variables.isScoringMode(ScoringMode.AMP))){
      m_arm.setShoulderGoal(Constants.ArmConstants.kShoulderAmp);
      m_arm.setElbowGoal(Constants.ArmConstants.kElbowAmp);
    }else /* if in SPEAKER scoring mode*/{
      //close shot
      if(m_variables.getDistanceToTarget() <= 5){
        m_arm.setShoulderGoal(ArmConstants.kShoulderHome);
        m_arm.setElbowGoal(ArmConstants.kElbowHome);
      }else{
        m_arm.setShoulderGoal(ArmConstants.kShoulderInterpolator.getInterpolatedValue(m_variables.getDistanceToTarget() * 12));//convert to inches
        m_arm.setElbowGoal(ArmConstants.kElbowInterpolator.getInterpolatedValue(m_variables.getDistanceToTarget() * 12));//convert to inches
      }
    }
  }

  public Command noWait(){
    noWait = true;
    return this;
  }

  @Override
  public void execute() {
    isFinished = Math.abs(m_arm.getShoulderAngle() - m_arm.getShoulderGoal()) <= ArmConstants.kTolerance
      && Math.abs(m_arm.getElbowAngle() - m_arm.getElbowGoal()) <= ArmConstants.kTolerance;
  }

  @Override
  public boolean isFinished() {
    return isFinished || noWait;
  }
}
