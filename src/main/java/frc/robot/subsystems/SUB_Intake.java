package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class SUB_Intake extends SubsystemBase{
    CANSparkMax m_groundIntakeMotor;
    CANSparkMax m_indexerMotor;
    double HighestCurrent = Double.MIN_VALUE;

    public SUB_Intake(){
        m_groundIntakeMotor = new CANSparkMax(IntakeConstants.kGroundIntakeMotorCANId, MotorType.kBrushless);
        m_groundIntakeMotor.setIdleMode(IdleMode.kBrake);
        m_indexerMotor.setSmartCurrentLimit(IntakeConstants.kGroundIntakeCurrentLimit);
        m_groundIntakeMotor.burnFlash();

        m_indexerMotor = new CANSparkMax(IntakeConstants.kIndexerMotorCANId, MotorType.kBrushless);
        m_indexerMotor.setIdleMode(IdleMode.kBrake);
        m_indexerMotor.setSmartCurrentLimit(IntakeConstants.kIndexerCurrentLimit);
        m_indexerMotor.burnFlash();
    }

    public void setGroundIntakePower(double p_power){
        m_groundIntakeMotor.set(p_power);
    }

    public void stopGroundIntake(){
        m_groundIntakeMotor.set(IntakeConstants.kIntakeOff);
    }

    public void setIndexerPower(double p_power){
        m_groundIntakeMotor.set(p_power);
    }

    public void stopIndexer(){
        m_groundIntakeMotor.set(IntakeConstants.kIntakeOff);
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("ground intake current", m_groundIntakeMotor.getOutputCurrent());
        if(m_groundIntakeMotor.getOutputCurrent() > HighestCurrent){
            HighestCurrent = m_groundIntakeMotor.getAppliedOutput();
        }
        SmartDashboard.putNumber("highest ground intake current", HighestCurrent);
    }
}
