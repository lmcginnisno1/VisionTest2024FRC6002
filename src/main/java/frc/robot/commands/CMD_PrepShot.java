package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.GlobalVariables;
import frc.robot.subsystems.SUB_Intake;

public class CMD_PrepShot extends Command{
    final SUB_Intake m_intake;
    final GlobalVariables m_variables;
    
    public CMD_PrepShot(SUB_Intake p_intake, GlobalVariables p_variables){
        m_intake = p_intake;
        m_variables = p_variables;
    }

    @Override
    public void initialize(){
        m_intake.prepShot();
        m_variables.setReadyToShoot(true);
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
