package frc.robot.subsystems;


import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class SUB_Arm extends SubsystemBase{
    CANSparkMax m_ShoulderMotor;
    AbsoluteEncoder m_ShoulderEncoder;

    CANSparkMax m_ElbowMotor;
    AbsoluteEncoder m_ElbowEncoder;

    SparkPIDController m_ShoulderPIDcontroller;
    SparkPIDController m_ElbowPIDcontroller;

    TrapezoidProfile m_ShoulderProfile;
    TrapezoidProfile.Constraints m_ShoulderConstraints;
    TrapezoidProfile.State m_ShoulderGoal;
    TrapezoidProfile.State m_ShoulderSetpoint;

    TrapezoidProfile m_ElbowProfile;
    TrapezoidProfile.Constraints m_ElbowConstraints;
    TrapezoidProfile.State m_ElbowGoal;
    TrapezoidProfile.State m_ElbowSetpoint;

    public SUB_Arm(){
        m_ShoulderMotor = new CANSparkMax(ArmConstants.kShoulderMotorCANId, MotorType.kBrushless);
        m_ShoulderPIDcontroller = m_ShoulderMotor.getPIDController();
        m_ShoulderEncoder = m_ShoulderMotor.getAbsoluteEncoder(Type.kDutyCycle);
        m_ShoulderEncoder.setInverted(false);
        m_ShoulderEncoder.setPositionConversionFactor(2 * Math.PI);
        m_ShoulderEncoder.setVelocityConversionFactor(2 * Math.PI / 60);
        m_ShoulderPIDcontroller.setPositionPIDWrappingEnabled(true);
        m_ShoulderPIDcontroller.setPositionPIDWrappingMinInput(0);
        m_ShoulderPIDcontroller.setPositionPIDWrappingMinInput(Math.PI * 2);

        m_ElbowMotor = new CANSparkMax(ArmConstants.kElbowMotorCANId, MotorType.kBrushless);
        m_ElbowPIDcontroller = m_ElbowMotor.getPIDController();
        m_ElbowEncoder = m_ElbowMotor.getAbsoluteEncoder(Type.kDutyCycle);
        m_ElbowEncoder.setInverted(false);
        m_ElbowEncoder.setPositionConversionFactor(2 * Math.PI);
        m_ElbowEncoder.setVelocityConversionFactor(2 * Math.PI / 60);
        m_ElbowPIDcontroller.setPositionPIDWrappingEnabled(true);
        m_ElbowPIDcontroller.setPositionPIDWrappingMinInput(0);
        m_ElbowPIDcontroller.setPositionPIDWrappingMinInput(Math.PI * 2);

        m_ShoulderPIDcontroller = m_ShoulderMotor.getPIDController();

        m_ShoulderPIDcontroller.setP(ArmConstants.kShoulderMotorP);
        m_ShoulderPIDcontroller.setI(ArmConstants.kShoulderMotorI);
        m_ShoulderPIDcontroller.setD(ArmConstants.kShoulderMotorD);
        m_ShoulderPIDcontroller.setFF(ArmConstants.kShoulderMotorFF);

        m_ElbowPIDcontroller.setP(ArmConstants.kElbowMotorP);
        m_ElbowPIDcontroller.setI(ArmConstants.kElbowMotorI);
        m_ElbowPIDcontroller.setD(ArmConstants.kElbowMotorD);
        m_ElbowPIDcontroller.setFF(ArmConstants.kElbowMotorFF);

        m_ShoulderMotor.setIdleMode(IdleMode.kBrake);
        m_ElbowMotor.setIdleMode(IdleMode.kBrake);

        m_ShoulderConstraints = new TrapezoidProfile.Constraints(ArmConstants.kShoulderMaxVelocity, ArmConstants.kShoulderMaxAcceleration);
        m_ShoulderSetpoint = new TrapezoidProfile.State(getShoulderPosition(), 0);
        m_ShoulderGoal = m_ShoulderSetpoint;
        m_ShoulderProfile = new TrapezoidProfile(m_ShoulderConstraints);

        m_ElbowConstraints = new TrapezoidProfile.Constraints(ArmConstants.kElbowMaxVelocity, ArmConstants.kElbowMaxAcceleration);
        m_ElbowSetpoint = new TrapezoidProfile.State(getElbowPosition(), 0);
        m_ElbowGoal = m_ElbowSetpoint;
        m_ElbowProfile = new TrapezoidProfile(m_ElbowConstraints);
    }

    public double getShoulderPosition(){
        return Math.toDegrees(m_ShoulderEncoder.getPosition());
    }

    private void setShoulderReference(double p_referece){
        m_ShoulderPIDcontroller.setReference(p_referece, ControlType.kPosition);
    }

    public void setShoulderGoal(double p_goal){
        m_ShoulderSetpoint = new TrapezoidProfile.State(getShoulderPosition(), 0);
        m_ShoulderGoal = new TrapezoidProfile.State(p_goal, 0);
    }

    public void ShoulderInit(){
        m_ShoulderSetpoint = new TrapezoidProfile.State(getShoulderPosition(), 0);
        m_ShoulderGoal = m_ShoulderSetpoint;
    }


    public double getElbowPosition(){
        return Math.toDegrees(m_ElbowEncoder.getPosition());
    }

    private void setElbowReference(double p_reference){
        double adjustedReference = Math.toRadians(p_reference);
        m_ElbowPIDcontroller.setReference(adjustedReference, ControlType.kPosition);
    }

    public void setElbowGoal(double p_goal){
        m_ElbowSetpoint = new TrapezoidProfile.State(getElbowPosition(), 0);
        m_ElbowGoal = new TrapezoidProfile.State(p_goal  - getShoulderPosition(), 0);
    }

    public void ElbowInit(){
        m_ElbowSetpoint = new TrapezoidProfile.State(getElbowPosition(), 0);
        m_ElbowGoal = m_ElbowSetpoint;
    }

    @Override
    public void periodic(){
        m_ShoulderSetpoint = m_ShoulderProfile.calculate(0.02, m_ShoulderSetpoint, m_ShoulderGoal);

        setShoulderReference(m_ShoulderSetpoint.position);


        m_ElbowSetpoint = m_ElbowProfile.calculate(0.02, m_ElbowSetpoint, m_ElbowGoal);

        setElbowReference(m_ElbowSetpoint.position);
    }
}