// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.SUB_Arm;

public class CMD_ArmHome extends Command {
  final SUB_Arm m_arm;
  boolean noWait;
  boolean isFinished;
  public CMD_ArmHome(SUB_Arm p_arm) {
    m_arm = p_arm;
  }

  @Override
  public void initialize() {
    m_arm.setShoulderGoal(ArmConstants.kShoulderHome);
    m_arm.setElbowGoal(ArmConstants.kElbowHome);
    noWait = false;
    isFinished = false;
  }

  public Command noWait(){
    noWait = true;
    return this;
  }

  @Override
  public void execute() {
    isFinished = Math.abs(m_arm.getShoulderGoal() - m_arm.getShoulderAngle()) <= ArmConstants.kTolerance &&
      Math.abs(m_arm.getElbowGoal() - m_arm.getElbowAngle()) <= ArmConstants.kTolerance;
  }

  @Override
  public boolean isFinished() {
    return noWait || isFinished;
  }
}
