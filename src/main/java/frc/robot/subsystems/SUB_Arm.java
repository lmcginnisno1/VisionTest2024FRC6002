package frc.robot.subsystems;


import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.utils.LinearInterpolator;

public class SUB_Arm extends SubsystemBase{
    final CANSparkMax m_MainShoulderMotor;
    final AbsoluteEncoder m_ShoulderEncoder;
    final CANSparkMax m_FollowerShoulderMotor;

    final CANSparkMax m_ElbowMotor;
    final AbsoluteEncoder m_ElbowEncoder;

    final SparkPIDController m_ShoulderPIDcontroller;
    final SparkPIDController m_ElbowPIDcontroller;

    final TrapezoidProfile m_ShoulderProfile;
    final TrapezoidProfile.Constraints m_ShoulderConstraints;
    TrapezoidProfile.State m_ShoulderGoal;
    TrapezoidProfile.State m_ShoulderSetpoint;

    final TrapezoidProfile m_ElbowProfile;
    final TrapezoidProfile.Constraints m_ElbowConstraints;
    TrapezoidProfile.State m_ElbowGoal;
    TrapezoidProfile.State m_ElbowSetpoint;

    final GlobalVariables m_variables;
    final LinearInterpolator m_ShoulderInterpolator = new LinearInterpolator(ArmConstants.kShoulderInterpolatorValues);
    final LinearInterpolator m_ElbowInterpolator = new LinearInterpolator(ArmConstants.kElbowInterpolatorValues);

    public SUB_Arm(GlobalVariables p_variables){
        m_variables = p_variables;

        m_MainShoulderMotor = new CANSparkMax(ArmConstants.kShoulderMotorMainCANId, MotorType.kBrushless);
        m_MainShoulderMotor.setIdleMode(IdleMode.kBrake);
        m_MainShoulderMotor.setSmartCurrentLimit(ArmConstants.kShoulderCurrentLimit);
        m_MainShoulderMotor.setInverted(true);
        m_ShoulderPIDcontroller = m_MainShoulderMotor.getPIDController();
        m_ShoulderPIDcontroller.setPositionPIDWrappingEnabled(true); 
        m_ShoulderPIDcontroller.setPositionPIDWrappingMinInput(-Math.PI);
        m_ShoulderPIDcontroller.setPositionPIDWrappingMaxInput(Math.PI);
        m_ShoulderEncoder = m_MainShoulderMotor.getAbsoluteEncoder(Type.kDutyCycle);
        m_ShoulderEncoder.setInverted(false);
        m_ShoulderEncoder.setPositionConversionFactor(ArmConstants.kShoulderPositionConversionFactor);
        m_ShoulderEncoder.setVelocityConversionFactor(ArmConstants.kShoulderVelocityConversionFactor);
        m_ShoulderPIDcontroller.setPositionPIDWrappingEnabled(true);
        m_ShoulderPIDcontroller.setPositionPIDWrappingMinInput(-Math.PI);
        m_ShoulderPIDcontroller.setPositionPIDWrappingMaxInput(Math.PI);
        m_ShoulderPIDcontroller.setFeedbackDevice(m_ShoulderEncoder);

        m_FollowerShoulderMotor = new CANSparkMax(ArmConstants.kShoulderFollowerCANId, MotorType.kBrushless);
        m_FollowerShoulderMotor.setIdleMode(IdleMode.kBrake);
        m_FollowerShoulderMotor.setSmartCurrentLimit(ArmConstants.kShoulderCurrentLimit);
        m_FollowerShoulderMotor.follow(m_MainShoulderMotor, true);

        m_ElbowMotor = new CANSparkMax(ArmConstants.kElbowMotorCANId, MotorType.kBrushless);
        m_ElbowMotor.setIdleMode(IdleMode.kBrake);
        m_ElbowMotor.setSmartCurrentLimit(ArmConstants.kElbowCurrentLimit);
        m_ElbowPIDcontroller = m_ElbowMotor.getPIDController();
        m_ElbowEncoder = m_ElbowMotor.getAbsoluteEncoder(Type.kDutyCycle);
        m_ElbowEncoder.setInverted(true);
        m_ElbowEncoder.setPositionConversionFactor(ArmConstants.kElbowPositionConversionFactor);
        m_ElbowEncoder.setVelocityConversionFactor(ArmConstants.kElbowVelocityConversionFactor);
        m_ElbowPIDcontroller.setPositionPIDWrappingEnabled(false);
        // m_ElbowPIDcontroller.setPositionPIDWrappingMinInput(-Math.PI);
        // m_ElbowPIDcontroller.setPositionPIDWrappingMaxInput(Math.PI);
        m_ElbowPIDcontroller.setFeedbackDevice(m_ElbowEncoder);

        m_ShoulderPIDcontroller.setP(ArmConstants.kShoulderP, 0);
        m_ShoulderPIDcontroller.setI(ArmConstants.kShoulderI, 0);
        m_ShoulderPIDcontroller.setD(ArmConstants.kShoulderD, 0);
        m_ShoulderPIDcontroller.setFF(ArmConstants.kShoulderFF, 0);

        m_ElbowPIDcontroller.setP(ArmConstants.kElbowP, 0);
        m_ElbowPIDcontroller.setI(ArmConstants.kElbowI, 0);
        m_ElbowPIDcontroller.setD(ArmConstants.kElbowD, 0);
        m_ElbowPIDcontroller.setFF(ArmConstants.kElbowFF, 0);

        m_MainShoulderMotor.setIdleMode(IdleMode.kBrake);
        m_ElbowMotor.setIdleMode(IdleMode.kBrake);

        m_ShoulderConstraints = new TrapezoidProfile.Constraints(ArmConstants.kShoulderMaxVelocityRadPerSecond, ArmConstants.kShoulderMaxAccelerationRadPerSecSquared);
        m_ShoulderSetpoint = new TrapezoidProfile.State(getShoulderAngleRad(), 0);
        m_ShoulderGoal = m_ShoulderSetpoint;
        m_ShoulderProfile = new TrapezoidProfile(m_ShoulderConstraints);

        m_ElbowConstraints = new TrapezoidProfile.Constraints(ArmConstants.kElbowMaxVelocityRadPerSecond, ArmConstants.kElbowMaxAccelerationRadPerSecSquared);
        m_ElbowSetpoint = new TrapezoidProfile.State(getElbowAngleRad(), 0);
        m_ElbowGoal = m_ElbowSetpoint;
        m_ElbowProfile = new TrapezoidProfile(m_ElbowConstraints);

        m_MainShoulderMotor.burnFlash();
        m_FollowerShoulderMotor.burnFlash();
        m_ElbowMotor.burnFlash();
    }

    public double getShoulderAngle(){
        return Math.toDegrees(MathUtil.angleModulus(m_ShoulderEncoder.getPosition()));    
    }

    public double getShoulderAngleRad(){
        return MathUtil.angleModulus(m_ShoulderEncoder.getPosition());
    }

    private double getShoulderVelocity(){
        return m_ShoulderEncoder.getVelocity();
    }

    private void setShoulderReference(double p_referece){
        m_ShoulderPIDcontroller.setReference(p_referece, ControlType.kPosition, 0);
    }

    public void setShoulderGoal(double p_goal){
        m_ShoulderSetpoint = new TrapezoidProfile.State(Math.toRadians(getShoulderAngleRad()), getShoulderVelocity());
        double ShoulderGoal = MathUtil.clamp(p_goal, Math.toRadians(-45), Math.toRadians(-10));
        m_ShoulderGoal = new TrapezoidProfile.State(Math.toRadians(ShoulderGoal), 0);
    }

    public double getShoulderGoal(){
        return Math.toDegrees(m_ShoulderGoal.position);
    }

    public void ShoulderInit(){
        m_ShoulderSetpoint = new TrapezoidProfile.State(getShoulderAngleRad(), 0);
        m_ShoulderGoal = m_ShoulderSetpoint;
    }


    public double getElbowAngle(){
        return Math.toDegrees(MathUtil.angleModulus(m_ElbowEncoder.getPosition()));
    }

    public double getElbowAngleRad(){
        return MathUtil.angleModulus(m_ElbowEncoder.getPosition());
    }

    private void setElbowReference(double p_reference){
        double adjustedReference = Math.toRadians(p_reference);
        m_ElbowPIDcontroller.setReference(adjustedReference, ControlType.kPosition, 0);
    }

    private double getElbowVelocity(){
        return m_ElbowEncoder.getVelocity();
    }

    public void setElbowGoal(double p_goal){
        m_ElbowSetpoint = new TrapezoidProfile.State(getElbowAngleRad(), getElbowVelocity());
        double ElbowGoal = MathUtil.clamp(p_goal, 8, 115);
        m_ElbowGoal = new TrapezoidProfile.State(Math.toRadians(ElbowGoal)  - getShoulderAngleRad(), 0);
    }

    public double getElbowGoal(){
        return Math.toDegrees(m_ElbowGoal.position);
    }

    public void ElbowInit(){
        m_ElbowSetpoint = new TrapezoidProfile.State(getElbowAngleRad(), 0);
        m_ElbowGoal = m_ElbowSetpoint;
    }

    @Override
    public void periodic(){
        m_ShoulderSetpoint = m_ShoulderProfile.calculate(0.02, m_ShoulderSetpoint, m_ShoulderGoal);

        setShoulderReference(m_ShoulderSetpoint.position);


        m_ElbowSetpoint = m_ElbowProfile.calculate(0.02, m_ElbowSetpoint, m_ElbowGoal);

        setElbowReference(m_ElbowSetpoint.position);

        telemetry();
    }

    public void telemetry(){
        SmartDashboard.putNumber("shoulder angle", getShoulderAngle());
        SmartDashboard.putNumber("elbow angle", getElbowAngle());
        SmartDashboard.putNumber("shoulder goal", Math.toDegrees(m_ShoulderGoal.position));
        SmartDashboard.putNumber("elbow goal", Math.toDegrees(m_ElbowGoal.position));
    }
}