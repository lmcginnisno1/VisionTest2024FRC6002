package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.SUB_Climber;

public class CMD_ClimberRelease extends InstantCommand {

  final SUB_Climber m_climber;

  public CMD_ClimberRelease(SUB_Climber p_climber) {
    m_climber = p_climber;
  }

  @Override
  public void initialize() {
    m_climber.setClosed();
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
