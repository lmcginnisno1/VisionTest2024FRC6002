package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.SUB_BottomShooter;

public class CMD_BottomShooterFeederSet extends Command {
  SUB_BottomShooter m_BottomShooter;
  double m_power;
  public CMD_BottomShooterFeederSet(SUB_BottomShooter p_BottomShooter, double p_power) {
    m_BottomShooter = p_BottomShooter;
    m_power = p_power;
  }

  @Override
  public void initialize() {
   m_BottomShooter.feederSet(ShooterConstants.kFeederFeed);
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
