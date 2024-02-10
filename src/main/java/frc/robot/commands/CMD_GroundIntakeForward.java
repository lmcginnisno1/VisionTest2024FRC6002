package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.SUB_Intake;

public class CMD_GroundIntakeForward extends Command {
  SUB_Intake m_intake;
  public CMD_GroundIntakeForward(SUB_Intake p_intake) {
    m_intake = p_intake;
  }

  @Override
  public void initialize() {
    m_intake.setGroundIntakePower(IntakeConstants.kIntakeForward);
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
