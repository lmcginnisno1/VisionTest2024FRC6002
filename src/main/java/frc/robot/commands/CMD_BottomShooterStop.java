package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.SUB_BottomShooter;

public class CMD_BottomShooterStop extends Command{
    SUB_BottomShooter m_BottomShooter;
    public CMD_BottomShooterStop(SUB_BottomShooter p_BottomShooter){
        m_BottomShooter = p_BottomShooter;
    }

    @Override
    public void initialize(){
        m_BottomShooter.setSetpoint(ShooterConstants.kShooterOff);
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
