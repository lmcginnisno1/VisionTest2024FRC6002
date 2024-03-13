// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.SUB_Intake;

public class CMD_IntakeReverse extends Command {
  final SUB_Intake m_intake;
  public CMD_IntakeReverse(SUB_Intake p_intake) {
    m_intake = p_intake;
  }

  @Override
  public void initialize() {
    m_intake.stopIntaking();
    m_intake.setGroundIntakePower(IntakeConstants.kIntakeReverse);
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
