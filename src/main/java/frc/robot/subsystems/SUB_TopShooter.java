package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants.ShooterConstants;

public class SUB_TopShooter extends PIDSubsystem{
    final CANSparkMax m_TopShooterMotor;
    final RelativeEncoder m_TopShooterEncoder;

    private final SimpleMotorFeedforward m_TopShooterFeedforward =
      new SimpleMotorFeedforward(
          ShooterConstants.kSTop, ShooterConstants.kVTop);

    public SUB_TopShooter(){
        super(new PIDController(ShooterConstants.kTopShooterP, ShooterConstants.kTopShooterI, ShooterConstants.kTopShooterD));
        getController().setTolerance(ShooterConstants.kShooterToleranceRPS);
        setSetpoint(0);
        m_TopShooterMotor = new CANSparkMax(ShooterConstants.kTopShooterMotorCANId, MotorType.kBrushless);
        m_TopShooterEncoder = m_TopShooterMotor.getEncoder();
        m_TopShooterMotor.setIdleMode(IdleMode.kCoast);
        m_TopShooterMotor.setSmartCurrentLimit(ShooterConstants.kShooterCurrentLimit);

        m_TopShooterMotor.burnFlash();
    }

    @Override
    protected void useOutput(double output, double setpoint) {
        m_TopShooterMotor.setVoltage(output + m_TopShooterFeedforward.calculate(setpoint));
    }

    @Override
    protected double getMeasurement() {
        return (double)m_TopShooterEncoder.getVelocity();
    }

    public boolean atSetpoint() {
        return m_controller.atSetpoint();
    }

    public void setVoltage(double p_voltage){
        m_TopShooterMotor.setVoltage(p_voltage);
    }

    public void telemetry(){
        SmartDashboard.putNumber("Top shooter motor velocity", m_TopShooterEncoder.getVelocity());
        SmartDashboard.putNumber("Top Shooter P", m_controller.getP());
        SmartDashboard.putNumber("Top Shooter I", m_controller.getI());
        SmartDashboard.putNumber("Top Shooter D", m_controller.getD());
        
        m_controller.setPID(SmartDashboard.getNumber("Top Shooter P", m_controller.getP()),
                            SmartDashboard.getNumber("Top Shooter I", m_controller.getI()),
                            SmartDashboard.getNumber("Top Shooter D", m_controller.getD())
        );
    }

    @Override
    public void periodic(){
        super.periodic();
        telemetry();
    }
}
