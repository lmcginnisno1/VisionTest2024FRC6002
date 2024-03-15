// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.GlobalVariables;
import frc.robot.subsystems.SUB_Arm;
import frc.robot.subsystems.SUB_Drivetrain;

public class CMD_AutoAim extends SequentialCommandGroup {
  public CMD_AutoAim(SUB_Drivetrain p_drive, SUB_Arm p_arm, GlobalVariables p_variables) {

    addCommands(
      new CMD_ArmAim(p_arm, p_variables).noWait(),
      new CMD_DriveAim(p_drive, p_variables)
    );
  }
}
