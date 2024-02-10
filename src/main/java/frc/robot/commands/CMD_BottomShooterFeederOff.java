package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SUB_BottomShooter;

public class CMD_BottomShooterFeederOff extends Command {
  SUB_BottomShooter m_BottomShooter;
  double m_power;
  public CMD_BottomShooterFeederOff(SUB_BottomShooter p_BottomShooter, double p_power) {
    m_BottomShooter = p_BottomShooter;
    m_power = p_power;
  }

  @Override
  public void initialize() {
   m_BottomShooter.feederOff();
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
