package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SUB_Intake;

public class CMD_GroundIntakeOff extends Command {
  final SUB_Intake m_intake;
  public CMD_GroundIntakeOff(SUB_Intake p_intake) {
    m_intake = p_intake;
  }

  @Override
  public void initialize() {
    m_intake.stopGroundIntake();
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
