// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.IntakeConstants;
import frc.robot.GlobalVariables.RobotState;
import frc.robot.RobotContainer;

public class CMD_IntakeDefault extends Command {
  final RobotContainer m_robot;
  public CMD_IntakeDefault(RobotContainer p_robot) {
    m_robot = p_robot;
    addRequirements(m_robot.m_intake);
  }

  @Override
  public void execute() {
    if(m_robot.m_variables.isRobotState(RobotState.ReadyToIntake) && m_robot.m_intake.getGroundIntakeCurrent() > 20){
            m_robot.m_intake.setGroundIntakePower(IntakeConstants.kIntakeForward / 2);
            m_robot.m_intake.setIndexerPower(IntakeConstants.kIntakeForward / 2);
        }
        if(m_robot.m_variables.isRobotState(RobotState.ReadyToIntake) && m_robot.m_intake.getIntakeSensor()){
            m_robot.m_intake.stopGroundIntake();
            m_robot.m_intake.stopIndexer();
            new SequentialCommandGroup(
              new InstantCommand(()-> m_robot.m_variables.setRobotState(RobotState.TransitioningToHome))
              ,new CMD_ArmHome(m_robot.m_arm)
              ,new InstantCommand(()-> m_robot.m_variables.setRobotState(RobotState.Stow))
              ,new InstantCommand(()-> m_robot.m_DriverControllerHI.setRumble(RumbleType.kBothRumble, 1))
              ,new WaitCommand(0.5)
              ,new InstantCommand(()-> m_robot.m_DriverControllerHI.setRumble(RumbleType.kBothRumble, 0))
            ).schedule();
        }
  }
}
