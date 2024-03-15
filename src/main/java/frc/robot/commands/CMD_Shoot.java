package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.GlobalVariables;
import frc.robot.subsystems.SUB_Intake;
import frc.robot.subsystems.SUB_Shooter;

public class CMD_Shoot extends Command{
    final SUB_Intake m_intake;
    final SUB_Shooter m_shooter;
    final GlobalVariables m_variables;
    boolean isFinished = false;
    boolean flywheelsAtSpeed = false;
    Timer m_ChargeTimer = new Timer();
    Timer m_powerOffTimer = new Timer();

    public CMD_Shoot(SUB_Intake p_intake, SUB_Shooter p_shooter, GlobalVariables p_variables){
        m_intake = p_intake;
        m_shooter = p_shooter;
        m_variables = p_variables;
    }

    @Override
    public void initialize(){
        isFinished = false;
        flywheelsAtSpeed = false;
        m_ChargeTimer.reset();
        m_ChargeTimer.stop();
        m_powerOffTimer.reset();
        m_ChargeTimer.stop();
    }

    @Override public void execute(){
        if(m_variables.getDistanceToTarget() <=5){
            m_ChargeTimer.start();
            if(m_ChargeTimer.get() > 0.2){
                m_intake.setIndexerPower(IntakeConstants.kIndexerForward);
                m_powerOffTimer.start();
            }
            if(m_ChargeTimer.get() > 0.2) {
                isFinished = true;
            }
        }else{
            m_ChargeTimer.start();
            if(m_ChargeTimer.get() > ShooterConstants.kShooterChargeTimeInterpolator.getInterpolatedValue(m_variables.getDistanceToTarget())){
                m_intake.setIndexerPower(IntakeConstants.kIndexerForward);
                m_powerOffTimer.start();
            }
        }

        isFinished = m_powerOffTimer.get() > 0.5;
    }

    @Override public void end(boolean interrupted){
        m_intake.setIndexerPower(IntakeConstants.kIndexerOff);
        m_shooter.enable();
        m_shooter.setSetpoint(ShooterConstants.kShooterOff);
        m_variables.setReadyToShoot(false);
    }

    @Override
    public boolean isFinished(){
        return isFinished;
    }
}
