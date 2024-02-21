package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SUB_BottomShooter;

public class CMD_BottomShooterSetVelocity extends Command{
    final SUB_BottomShooter m_BottomShooter;
    double m_power;
    public CMD_BottomShooterSetVelocity(SUB_BottomShooter p_BottomShooter, double p_power){
        m_BottomShooter = p_BottomShooter;
        m_power = p_power;
    }

    @Override
    public void initialize(){
        m_BottomShooter.setSetpoint(m_power);
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
