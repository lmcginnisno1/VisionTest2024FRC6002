package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ClimberConstants;

public class SUB_Climber extends SubsystemBase{
    CANSparkMax m_WinchMotor;
    SparkPIDController m_WinchPIDcontroller;
    RelativeEncoder m_WinchEncoder;
    
    TrapezoidProfile m_WinchProfile;
    TrapezoidProfile.Constraints m_WinchConstraints;
    TrapezoidProfile.State m_WinchGoal;
    TrapezoidProfile.State m_WinchSetpoint;

    PWM m_ClimberServo = new PWM(ClimberConstants.kClimberServoPWMPort);

    public SUB_Climber(){
        m_WinchMotor = new CANSparkMax(ClimberConstants.kClimberMotorCANId, MotorType.kBrushless);
        m_WinchEncoder = m_WinchMotor.getEncoder();

        m_WinchPIDcontroller = m_WinchMotor.getPIDController();
        
        m_WinchPIDcontroller.setP(ClimberConstants.kWinchMotorP);
        m_WinchPIDcontroller.setI(ClimberConstants.kWinchMotorI);
        m_WinchPIDcontroller.setD(ClimberConstants.kWinchMotorD);
        m_WinchPIDcontroller.setFF(ClimberConstants.kWinchMotorFF);

        m_WinchMotor.setIdleMode(IdleMode.kBrake);

        m_WinchConstraints = new TrapezoidProfile.Constraints(ClimberConstants.kWinchMaxVelocity, ClimberConstants.kWinchMaxAcceleration);
        m_WinchSetpoint = new TrapezoidProfile.State(getWinchPosition(), 0);
        m_WinchGoal = m_WinchSetpoint;

        m_WinchProfile = new TrapezoidProfile(m_WinchConstraints);
        m_WinchMotor.burnFlash();
    }

    public double getWinchPosition(){
        return m_WinchEncoder.getPosition();
    }

    public void setWinchReference(double p_referece){
        m_WinchPIDcontroller.setReference(p_referece, ControlType.kPosition);
    }

    public void setWinchGoal(double p_goal){
        m_WinchSetpoint = new TrapezoidProfile.State(getWinchPosition(), 0);
        m_WinchGoal = new TrapezoidProfile.State(p_goal, 0);
    }

    public void WinchInit(){
      m_WinchSetpoint = new TrapezoidProfile.State(getWinchPosition(), 0); 
      m_WinchGoal = m_WinchSetpoint;

      m_WinchSetpoint = m_WinchProfile.calculate(0.02, m_WinchSetpoint, m_WinchGoal);

      setWinchReference(m_WinchSetpoint.position);
    }

    public void setClimbServoUnhooked(){
        m_ClimberServo.setPosition(ClimberConstants.kClimberServoUnhooked);
    }

    public void setClimbServoHooked(){
        m_ClimberServo.setPosition(ClimberConstants.kClimberServoHooked);
    }
}