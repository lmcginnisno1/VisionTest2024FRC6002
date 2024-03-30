package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.GlobalVariables;
import frc.robot.Constants.IntakeConstants;

public class SUB_Intake extends SubsystemBase{
    final GlobalVariables m_variables;
    final CANSparkMax m_mainGroundIntakeMotor;
    final CANSparkMax m_followerGroundIntakeMotor;
    final SparkPIDController m_GroundIntakePIDcontroller;
    final CANSparkMax m_indexerMotor;
    final SparkPIDController m_IndexerPIDcontroller;
    final DigitalInput m_indexerSensor = new DigitalInput(IntakeConstants.kIndexerSensorDigitalPort);
    final RelativeEncoder m_indexerEncoder;
    final RelativeEncoder m_intakeEncoder;
    // boolean firstloop = true;
    // double P = 0, I = 0, D = 0, FF = 0, kV = 0, goal = 0, setpoint = 0;

    public SUB_Intake(GlobalVariables p_variables){
        m_variables = p_variables;
        m_mainGroundIntakeMotor = new CANSparkMax(IntakeConstants.kMainGroundIntakeMotorCANId, MotorType.kBrushless);
        m_mainGroundIntakeMotor.setInverted(false);
        m_GroundIntakePIDcontroller = m_mainGroundIntakeMotor.getPIDController();
        m_GroundIntakePIDcontroller.setP(IntakeConstants.kGroundIntakeP);
        m_GroundIntakePIDcontroller.setI(IntakeConstants.kGroundIntakeI);
        m_GroundIntakePIDcontroller.setD(IntakeConstants.kGroundIntakeD);
        m_GroundIntakePIDcontroller.setFF(IntakeConstants.kGroundIntakeFF);
        m_mainGroundIntakeMotor.setIdleMode(IdleMode.kCoast);
        m_mainGroundIntakeMotor.setSmartCurrentLimit(IntakeConstants.kGroundIntakeCurrentLimit);
        m_intakeEncoder = m_mainGroundIntakeMotor.getEncoder();
        m_mainGroundIntakeMotor.burnFlash();

        m_followerGroundIntakeMotor = new CANSparkMax(IntakeConstants.kFollowerGroundIntakeMotorCANId, MotorType.kBrushless);
        m_followerGroundIntakeMotor.setIdleMode(IdleMode.kCoast);
        m_followerGroundIntakeMotor.setSmartCurrentLimit(IntakeConstants.kGroundIntakeCurrentLimit);
        m_followerGroundIntakeMotor.burnFlash();
        m_followerGroundIntakeMotor.follow(m_mainGroundIntakeMotor, true);

        m_indexerMotor = new CANSparkMax(IntakeConstants.kIndexerMotorCANId, MotorType.kBrushless);
        m_IndexerPIDcontroller = m_indexerMotor.getPIDController();
        m_IndexerPIDcontroller.setP(IntakeConstants.kIndexerP);
        m_IndexerPIDcontroller.setI(IntakeConstants.kIndexerI);
        m_IndexerPIDcontroller.setD(IntakeConstants.kIndexerD);
        m_IndexerPIDcontroller.setFF(IntakeConstants.kIndexerFF);
        m_indexerMotor.setIdleMode(IdleMode.kBrake);
        m_indexerMotor.setSmartCurrentLimit(IntakeConstants.kIndexerCurrentLimit);
        m_indexerMotor.setInverted(true);
        m_indexerMotor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen).enableLimitSwitch(false);
        m_indexerMotor.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen).enableLimitSwitch(false);
        m_indexerMotor.burnFlash();
        m_indexerEncoder = m_indexerMotor.getEncoder();
    }

    public void setGroundIntakeVelocity(double p_velocity){
        m_GroundIntakePIDcontroller.setReference(p_velocity, ControlType.kVelocity);
    }

    public void stopGroundIntake(){
        m_GroundIntakePIDcontroller.setReference(0, ControlType.kVelocity);
    }

    public void setIndexerVelocity(double p_velocity){
        m_IndexerPIDcontroller.setReference(p_velocity, ControlType.kVelocity);
    }

    public void stopIndexer(){
        m_IndexerPIDcontroller.setReference(IntakeConstants.kIntakeOff, ControlType.kVelocity);
    }

    public boolean getIntakeSensor(){
        return !m_indexerSensor.get();
    }

    public double getGroundIntakeCurrent(){
        return m_mainGroundIntakeMotor.getOutputCurrent();
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("ground intake current", m_mainGroundIntakeMotor.getOutputCurrent());
        SmartDashboard.putBoolean("inake sensor detected", getIntakeSensor());

        // if(firstloop){
        //     SmartDashboard.putNumber("intake P", P);
        //     SmartDashboard.putNumber("intake I", I);
        //     SmartDashboard.putNumber("intake D", D);
        //     SmartDashboard.putNumber("intake FF", FF);
        //     SmartDashboard.putNumber("intake goal", goal);
        //     SmartDashboard.putNumber("intake setpoint", setpoint);
        //     firstloop = false;
        // }
        // P = SmartDashboard.getNumber("intake P", P);
        // I = SmartDashboard.getNumber("intake I", I);
        // D = SmartDashboard.getNumber("intake D", D);
        // FF = SmartDashboard.getNumber("intake FF", FF);
        // goal = SmartDashboard.getNumber("intake goal", goal);
        // m_GroundIntakePIDcontroller.setP(P);
        // m_GroundIntakePIDcontroller.setI(I);
        // m_GroundIntakePIDcontroller.setD(D);
        // m_GroundIntakePIDcontroller.setFF(FF);
        // m_GroundIntakePIDcontroller.setReference(goal, ControlType.kVelocity);
        // setpoint = m_intakeEncoder.getVelocity();
        // SmartDashboard.putNumber("intake setpoint", setpoint);
    }
}
