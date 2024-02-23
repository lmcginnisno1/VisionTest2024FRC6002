package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.GlobalVariables;
import frc.robot.subsystems.SUB_Intake;
import frc.robot.subsystems.SUB_Shooter;

public class CMD_Shoot extends Command{
    final SUB_Intake m_intake;
    final SUB_Shooter m_shooter;
    final GlobalVariables m_globalVariables;
    boolean isFinished = false;
    boolean flywheelsAtSpeed = false;
    Timer m_Timer = new Timer();

    public CMD_Shoot(SUB_Intake p_intake, SUB_Shooter p_shooter, GlobalVariables p_variables){
        m_intake = p_intake;
        m_shooter = p_shooter;
        m_globalVariables = p_variables;
    }

    @Override
    public void initialize(){
        isFinished = false;
        flywheelsAtSpeed = false;
        m_Timer.stop();
        m_Timer.reset();
    }

    @Override public void execute(){
        double [] shooterVelocites = m_shooter.getShooterVelocites();
        flywheelsAtSpeed = Math.abs(shooterVelocites[0] - m_shooter.getInterpolatedRPM()) < 50 &&
                           Math.abs(shooterVelocites[1] - m_shooter.getInterpolatedRPM()) < 50;
        if(flywheelsAtSpeed){
            m_Timer.start();
            m_intake.setIndexerPower(1);
            if(m_Timer.get() > 1){
                m_intake.setIndexerPower(0);
                isFinished = true;
            }
        }
    }

    @Override
    public boolean isFinished(){
        return isFinished;
    }
}
