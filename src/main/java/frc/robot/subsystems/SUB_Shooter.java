package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.GlobalVariables;
import frc.robot.Constants.LedConstants;
import frc.robot.Constants.ShooterConstants;
import frc.utils.LinearInterpolator;

public class SUB_Shooter extends SubsystemBase{
    CANSparkMax m_shooterFeederMotor;
    final SUB_TopShooter m_topShooter;
    final SUB_BottomShooter m_bottomShooter;
    final SUB_Led m_led;
    final GlobalVariables m_variables;
    final LinearInterpolator m_shooterInterpolator = new LinearInterpolator(ShooterConstants.kShooterInterpolatorValues);
    double interpolatedRPM = 0;

    public SUB_Shooter(SUB_TopShooter p_TopShooter, SUB_BottomShooter p_BottomShooter, SUB_Led p_led, GlobalVariables p_variables){
        m_topShooter = p_TopShooter;
        m_bottomShooter = p_BottomShooter;
        m_led = p_led;
        m_variables = p_variables;
    }

    public double[] getShooterVelocites(){
        //returns the velocites of both shooters, top shooter velocity is at index 0, then bottom shooter velocity at index 1
        double[] velocites = {m_topShooter.getMeasurement(), m_bottomShooter.getMeasurement()};
        return velocites;
    }

    public void setShooterVoltage(double p_voltage){
        m_topShooter.setVoltage(p_voltage);
        m_bottomShooter.setVoltage(p_voltage);
    }

    public void setSetpoint(double p_setpoint){
        m_topShooter.setSetpoint(p_setpoint);
        m_bottomShooter.setSetpoint(p_setpoint);
    }

    public void enable(){
        m_topShooter.enable();
        m_bottomShooter.enable();
    }

    public void disable(){
        m_topShooter.disable();
        m_bottomShooter.disable();
    }

    @Override
    public void periodic(){
        if(m_variables.ReadyToShoot()){
            m_led.setLedColors(LedConstants.kBrightGreen, LedConstants.kBrightGreen);
            m_led.setPattern(LedConstants.kChasePattern);
            m_led.setPatternDuration(1 / (getShooterVelocites()[1] / 500));
        }
    }

    public double getInterpolatedRPM(){
        return interpolatedRPM;
    }

    public void stopShooter(){
        m_topShooter.setSetpoint(0);
        m_bottomShooter.setSetpoint(0);
    }
}
