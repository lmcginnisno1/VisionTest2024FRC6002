package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SUB_Shooter extends SubsystemBase{
    CANSparkMax m_ShooterFeederMotor;
    SUB_TopShooter m_TopShooter;
    SUB_BottomShooter m_BottomShooter;

    public SUB_Shooter(SUB_TopShooter p_TopShooter, SUB_BottomShooter p_BottomShooter){
        m_TopShooter = p_TopShooter;
        m_BottomShooter = p_BottomShooter;
    }
}
