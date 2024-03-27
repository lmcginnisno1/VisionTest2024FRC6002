package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class SUB_Intake extends SubsystemBase{
    final GlobalVariables m_variables;
    final CANSparkMax m_mainGroundIntakeMotor;
    final CANSparkMax m_followerGroundIntakeMotor;
    final SparkPIDController m_GroundIntakePIDcontroller;
    final CANSparkMax m_indexerMotor;
    final DigitalInput m_indexerSensor = new DigitalInput(IntakeConstants.kIndexerSensorDigitalPort);
    final RelativeEncoder m_indexerEncoder;
    final SparkPIDController m_indexerController;
    boolean intaking = false;

    public SUB_Intake(GlobalVariables p_variables){
        m_variables = p_variables;
        m_mainGroundIntakeMotor = new CANSparkMax(IntakeConstants.kMainGroundIntakeMotorCANId, MotorType.kBrushless);
        m_mainGroundIntakeMotor.setInverted(true);
        m_GroundIntakePIDcontroller = m_mainGroundIntakeMotor.getPIDController();
        m_GroundIntakePIDcontroller.setP(IntakeConstants.kGroundIntakeP);
        m_GroundIntakePIDcontroller.setI(IntakeConstants.kGroundIntakeI);
        m_GroundIntakePIDcontroller.setD(IntakeConstants.kGroundIntakeD);
        m_GroundIntakePIDcontroller.setFF(IntakeConstants.kGroundIntakeFF);
        m_mainGroundIntakeMotor.setIdleMode(IdleMode.kCoast);
        m_mainGroundIntakeMotor.setSmartCurrentLimit(IntakeConstants.kGroundIntakeCurrentLimit);
        m_mainGroundIntakeMotor.burnFlash();

        m_followerGroundIntakeMotor = new CANSparkMax(IntakeConstants.kFollowerGroundIntakeMotorCANId, MotorType.kBrushless);
        m_followerGroundIntakeMotor.setIdleMode(IdleMode.kCoast);
        m_followerGroundIntakeMotor.setSmartCurrentLimit(IntakeConstants.kGroundIntakeCurrentLimit);
        m_followerGroundIntakeMotor.burnFlash();
        m_followerGroundIntakeMotor.follow(m_mainGroundIntakeMotor, false);

        m_indexerMotor = new CANSparkMax(IntakeConstants.kIndexerMotorCANId, MotorType.kBrushless);
        m_indexerMotor.setIdleMode(IdleMode.kBrake);
        m_indexerMotor.setSmartCurrentLimit(IntakeConstants.kIndexerCurrentLimit);
        m_indexerMotor.setInverted(true);
        m_indexerMotor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen).enableLimitSwitch(false);
        m_indexerMotor.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen).enableLimitSwitch(false);
        m_indexerMotor.burnFlash();

        m_indexerEncoder = m_indexerMotor.getEncoder();
        m_indexerEncoder.setPositionConversionFactor(1/40);
        m_indexerController = m_indexerMotor.getPIDController();
        m_indexerController.setFF(0.001);
        m_indexerController.setP(0.001);
    }

    public void setGroundIntakePower(double p_power){
        m_mainGroundIntakeMotor.set(p_power);
    }

    public void stopGroundIntake(){
        m_mainGroundIntakeMotor.set(IntakeConstants.kIntakeOff);
    }

    public void setIndexerPower(double p_power){
        m_mainGroundIntakeMotor.set(p_power);
    }

    public void stopIndexer(){
        m_mainGroundIntakeMotor.set(IntakeConstants.kIntakeOff);
    }

    public boolean getIntakeSensor(){
        return m_indexerSensor.get();
    }

    public double getGroundIntakeCurrent(){
        return m_mainGroundIntakeMotor.getOutputCurrent();
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("ground intake current", m_mainGroundIntakeMotor.getOutputCurrent());
    }
}
