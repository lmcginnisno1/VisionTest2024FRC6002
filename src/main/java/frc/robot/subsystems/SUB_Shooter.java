package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.GlobalVariables;
import frc.robot.Constants.ShooterConstants;
import frc.utils.LinearInterpolator;

public class SUB_Shooter extends SubsystemBase{
    CANSparkMax m_ShooterFeederMotor;
    final SUB_TopShooter m_TopShooter;
    final SUB_BottomShooter m_BottomShooter;
    final GlobalVariables m_variables;
    final LinearInterpolator m_shooterInterpolator = new LinearInterpolator(ShooterConstants.kShooterInterpolatorValues);
    double interpolatedRPM = 0;

    public SUB_Shooter(SUB_TopShooter p_TopShooter, SUB_BottomShooter p_BottomShooter, GlobalVariables p_variables){
        m_TopShooter = p_TopShooter;
        m_BottomShooter = p_BottomShooter;
        m_variables = p_variables;
    }

    public double[] getShooterVelocites(){
        //returns the velocites of both shooters, top shooter velocity is at index 0, then bottom shooter velocity at index 1
        double[] velocites = {m_TopShooter.getMeasurement(), m_BottomShooter.getMeasurement()};
        return velocites;
    }

    @Override
    public void periodic(){
        if(m_variables.ReadyToShoot()){
            interpolatedRPM = m_shooterInterpolator.getInterpolatedValue(m_variables.getDistanceToTarget());
            m_TopShooter.setSetpoint(interpolatedRPM);
            m_BottomShooter.setSetpoint(interpolatedRPM);
        }
    }

    public double getInterpolatedRPM(){
        return interpolatedRPM;
    }

    public void stopShooter(){
        m_TopShooter.setSetpoint(0);
        m_BottomShooter.setSetpoint(0);
    }
}
