package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.GlobalVariables;
import frc.robot.Constants.LedConstants;
import frc.robot.subsystems.SUB_Intake;
import frc.robot.subsystems.SUB_Led;

public class CMD_PrepShot extends Command{
    final SUB_Intake m_intake;
    final SUB_Led m_led;
    final GlobalVariables m_variables;
    
    public CMD_PrepShot(SUB_Intake p_intake, SUB_Led p_led, GlobalVariables p_variables){
        m_intake = p_intake;
        m_led = p_led;
        m_variables = p_variables;
    }

    @Override
    public void initialize(){
        m_intake.prepShot();
        m_variables.setReadyToShoot(true);
        m_led.setPattern(LedConstants.kChasePattern);
        m_led.setLedColors(LedConstants.kBrightGreen, LedConstants.kBrightGreen);
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
