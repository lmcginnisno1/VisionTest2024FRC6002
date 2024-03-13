package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.GlobalVariables;
import frc.robot.Constants.LedConstants;
import frc.robot.Constants.ShooterConstants;
import frc.utils.LinearInterpolator;

public class SUB_Shooter extends SubsystemBase{
    CANSparkMax m_ShooterFeederMotor;
    final SUB_TopShooter m_TopShooter;
    final SUB_BottomShooter m_BottomShooter;
    final SUB_Led m_led;
    final GlobalVariables m_variables;
    final LinearInterpolator m_shooterInterpolator = new LinearInterpolator(ShooterConstants.kShooterInterpolatorValues);
    double interpolatedRPM = 0;

    public SUB_Shooter(SUB_TopShooter p_TopShooter, SUB_BottomShooter p_BottomShooter, SUB_Led p_led, GlobalVariables p_variables){
        m_TopShooter = p_TopShooter;
        m_BottomShooter = p_BottomShooter;
        m_led = p_led;
        m_variables = p_variables;
    }

    public double[] getShooterVelocites(){
        //returns the velocites of both shooters, top shooter velocity is at index 0, then bottom shooter velocity at index 1
        double[] velocites = {m_TopShooter.getMeasurement(), m_BottomShooter.getMeasurement()};
        return velocites;
    }

    public void setShooterVoltage(double p_voltage){
        m_TopShooter.setVoltage(p_voltage);
        m_BottomShooter.setVoltage(p_voltage);
    }

    public void setSetpoint(double p_setpoint){
        m_TopShooter.setSetpoint(p_setpoint);
        m_BottomShooter.setSetpoint(p_setpoint);
    }

    public void enable(){
        m_TopShooter.enable();
        m_BottomShooter.enable();
    }

    public void disable(){
        m_TopShooter.disable();
        m_BottomShooter.disable();
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
        m_TopShooter.setSetpoint(0);
        m_BottomShooter.setSetpoint(0);
    }
}
