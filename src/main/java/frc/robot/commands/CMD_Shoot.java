package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.GlobalVariables;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.SUB_Intake;
import frc.robot.subsystems.SUB_Shooter;

public class CMD_Shoot extends Command{
    final SUB_Intake m_intake;
    final SUB_Shooter m_shooter;
    final GlobalVariables m_variables;
    boolean isFinished = false;
    boolean flywheelsAtSpeed = false;

    public CMD_Shoot(SUB_Intake p_intake, SUB_Shooter p_shooter, GlobalVariables p_variables){
        m_intake = p_intake;
        m_shooter = p_shooter;
        m_variables = p_variables;
    }

    @Override
    public void initialize(){
        isFinished = false;
        flywheelsAtSpeed = false;
    }

    @Override public void execute(){
        if(m_shooter.m_topShooter.atSetpoint() && m_shooter.m_bottomShooter.atSetpoint()){
            flywheelsAtSpeed = true;
        }
        if(flywheelsAtSpeed){
            new SequentialCommandGroup(
                new InstantCommand(()-> m_intake.setIndexerVelocity(IntakeConstants.kIndexerForward))
                ,new InstantCommand(()-> m_intake.setGroundIntakeVelocity(IntakeConstants.kIntakeForward))
                ,new WaitCommand(1)
                ,new InstantCommand(()-> m_intake.setIndexerVelocity(IntakeConstants.kIndexerOff))
                ,new InstantCommand(()-> m_intake.setGroundIntakeVelocity(IntakeConstants.kIntakeOff))
                ,new InstantCommand(()-> m_shooter.setSetpoint(ShooterConstants.kShooterOff))
            ).schedule();
        }
    }

    @Override public void end(boolean interrupted){
        m_intake.setIndexerVelocity(IntakeConstants.kIndexerOff);
        m_intake.setGroundIntakeVelocity(IntakeConstants.kIntakeOff);
        m_shooter.setSetpoint(ShooterConstants.kShooterOff);
        m_variables.setReadyToShoot(false);
    }

    @Override
    public boolean isFinished(){
        return isFinished;
    }
}
