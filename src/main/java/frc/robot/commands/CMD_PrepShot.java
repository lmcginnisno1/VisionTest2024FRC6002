package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SUB_Intake;

public class CMD_PrepShot extends Command{
    final SUB_Intake m_intake;
    
    public CMD_PrepShot(SUB_Intake p_intake){
        m_intake = p_intake;
    }

    @Override
    public void initialize(){
        m_intake.prepShot();
    }
}
