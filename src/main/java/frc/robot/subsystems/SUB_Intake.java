package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.GlobalVariables;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.LedConstants;
import frc.robot.GlobalVariables.IntakeMode;

public class SUB_Intake extends SubsystemBase{
    final SUB_Led m_Led;
    final GlobalVariables m_variables;
    CANSparkMax m_groundIntakeMotor;
    CANSparkMax m_indexerMotor;
    DigitalInput m_indexerSensor = new DigitalInput(IntakeConstants.kIndexerSensorDigitalPort);
    RelativeEncoder m_indexerEncoder;
    SparkPIDController m_indexerController;
    boolean intaking = false;

    public SUB_Intake(SUB_Led p_Led, GlobalVariables p_variables){
        m_Led = p_Led;
        m_variables = p_variables;
        m_groundIntakeMotor = new CANSparkMax(IntakeConstants.kGroundIntakeMotorCANId, MotorType.kBrushless);
        m_groundIntakeMotor.setIdleMode(IdleMode.kBrake);
        m_indexerMotor.setSmartCurrentLimit(IntakeConstants.kGroundIntakeCurrentLimit);
        m_groundIntakeMotor.burnFlash();

        m_indexerMotor = new CANSparkMax(IntakeConstants.kIndexerMotorCANId, MotorType.kBrushless);
        m_indexerMotor.setIdleMode(IdleMode.kBrake);
        m_indexerMotor.setSmartCurrentLimit(IntakeConstants.kIndexerCurrentLimit);
        m_indexerMotor.burnFlash();

        m_indexerEncoder = m_indexerMotor.getEncoder();
        m_indexerEncoder.setPositionConversionFactor(1/40);
        m_indexerController = m_indexerMotor.getPIDController();
        m_indexerController.setFF(0.001);
        m_indexerController.setP(0.001);
    }

    public void setGroundIntakePower(double p_power){
        if(p_power > 0){
            intaking = true;
            setIndexerPower(p_power);
        }
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

    public boolean getIntakeSensor(){
        return m_indexerSensor.get();
    }

    public void startIntaking(){
        intaking = true;
    }

    public void stopIntaking(){
        intaking = false;
    }

    public void prepShot(){
        m_indexerEncoder.setPosition(0);
        m_indexerController.setReference(-10, ControlType.kPosition);
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("ground intake current", m_groundIntakeMotor.getOutputCurrent());
        if(m_groundIntakeMotor.getAppliedOutput() > 0 && getIntakeSensor() && intaking){
            stopGroundIntake();
            stopIndexer();
            m_Led.setPattern(LedConstants.kSolidColorPattern);
            m_Led.setLedColors(LedConstants.kBrightGreen, LedConstants.kBrightGreen);
            intaking = false;
        }
        if(intaking && m_variables.getIntakeMode() == IntakeMode.GROUND){
            m_Led.setPattern(LedConstants.kChasePattern);
            m_Led.setLedColors(LedConstants.kBrightOrange, LedConstants.kBrightOrange);
        }else if(intaking && m_variables.getIntakeMode() == IntakeMode.SOURCE){
            m_Led.setPattern(LedConstants.kWavePattern);
            m_Led.setLedColors(LedConstants.kBrightOrange, LedConstants.kMaroon);
        }
    }
}
