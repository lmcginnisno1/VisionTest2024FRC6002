// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.GlobalVariables;
import frc.robot.subsystems.SUB_Drivetrain;

public class CMD_DriveAim extends Command {
  final SUB_Drivetrain m_drivetrain;
  final GlobalVariables m_variables;
  double angleToTarget;
  boolean isFinished;
  public CMD_DriveAim(SUB_Drivetrain p_drivetrain, GlobalVariables p_variables) {
    m_drivetrain = p_drivetrain;
    m_variables = p_variables;
  }

  @Override
  public void initialize() {
    isFinished = false;
  }

  @Override
  public void execute() {
    double rot = 0;
    double heading_error =  m_variables.getRobotPose().getRotation().getDegrees() - m_variables.getAngleToTarget();

    if (Math.abs(heading_error) >= 1) {
      rot = heading_error * 0.002;
    }else{
      isFinished = true;
    }

    m_drivetrain.drive(0, rot, 0, false, true);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
