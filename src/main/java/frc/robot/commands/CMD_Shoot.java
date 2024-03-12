package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.GlobalVariables;
import frc.robot.Constants.LedConstants;
import frc.robot.subsystems.SUB_Intake;
import frc.robot.subsystems.SUB_Led;
import frc.robot.subsystems.SUB_Shooter;

public class CMD_Shoot extends Command{
    final SUB_Intake m_intake;
    final SUB_Shooter m_shooter;
    final SUB_Led m_led;
    final GlobalVariables m_variables;
    boolean isFinished = false;
    boolean flywheelsAtSpeed = false;
    Timer m_Timer = new Timer();

    public CMD_Shoot(SUB_Intake p_intake, SUB_Shooter p_shooter, SUB_Led p_led, GlobalVariables p_variables){
        m_intake = p_intake;
        m_shooter = p_shooter;
        m_led = p_led;
        m_variables = p_variables;
    }

    @Override
    public void initialize(){
        isFinished = false;
        flywheelsAtSpeed = false;
        m_Timer.stop();
        m_Timer.reset();
    }

    @Override public void execute(){
        m_led.setLedColors(LedConstants.kBrightGreen, LedConstants.kBrightGreen);
        m_led.setPattern(LedConstants.kChasePattern);
        m_led.setPatternDuration(1 / (m_shooter.getShooterVelocites()[1] / 500));
        if(m_variables.getDistanceToTarget() <=5){
            
        }
    }

    @Override
    public boolean isFinished(){
        return isFinished;
    }
}
