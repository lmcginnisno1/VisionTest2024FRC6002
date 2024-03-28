// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.GlobalVariables.RobotState;

public class CMD_HandleReadyToShoot extends Command {
  final RobotContainer m_robot;
  public CMD_HandleReadyToShoot(RobotContainer p_robot) {
    m_robot = p_robot;
  }

  @Override
  public void initialize() {
    if(m_robot.m_variables.isRobotState(RobotState.Home)
          || m_robot.m_variables.isRobotState(RobotState.Stow)
          || m_robot.m_variables.isRobotState(RobotState.ReadyToShoot)
      ){ /*these are valid states, allow to continue*/} else return;

    if(m_robot.m_variables.isRobotState(RobotState.Home)){
      new SequentialCommandGroup(
        new InstantCommand(()-> m_robot.m_variables.setRobotState(RobotState.ReadyToIntake))
        ,new CMD_IntakeForward(m_robot.m_intake)
      ).schedule();
      return;
    }

    if(m_robot.m_variables.isRobotState(RobotState.Stow)){
      new SequentialCommandGroup(
        new InstantCommand(()-> m_robot.m_variables.setRobotState(RobotState.TransitioningToShoot))
        ,new CMD_RevShooter(m_robot.m_shooter, m_robot.m_variables)
        ,new CMD_ArmAim(m_robot.m_arm, m_robot.m_variables).noWait()
        ,new CMD_DriveAim(m_robot.m_robotDrive, m_robot.m_variables)
        ,new InstantCommand(()-> m_robot.m_variables.setRobotState(RobotState.ReadyToShoot))
        ,new WaitCommand(1)
        ,new CMD_Shoot(m_robot.m_intake, m_robot.m_shooter, m_robot.m_variables)
        ,new InstantCommand(()-> m_robot.m_variables.setRobotState(RobotState.TransitioningToHome))
        ,new CMD_ArmHome(m_robot.m_arm)
        ,new InstantCommand(()-> m_robot.m_variables.setRobotState(RobotState.Home))
      ).schedule();
      return;
    }
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
