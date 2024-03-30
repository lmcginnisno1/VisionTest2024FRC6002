package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants.ShooterConstants;

public class SUB_TopShooter extends PIDSubsystem{
    final CANSparkMax m_topShooterMotor;
    final RelativeEncoder m_topShooterEncoder;
    boolean firstloop = true;

    private SimpleMotorFeedforward m_topShooterFeedforward =
      new SimpleMotorFeedforward(
          ShooterConstants.kSTop, ShooterConstants.kVTop);

    public SUB_TopShooter(){
        super(new PIDController(ShooterConstants.kTopShooterP, ShooterConstants.kTopShooterI, ShooterConstants.kTopShooterD));
        getController().setTolerance(ShooterConstants.kShooterToleranceRPS);
        setSetpoint(0);
        m_topShooterMotor = new CANSparkMax(ShooterConstants.kTopShooterMotorCANId, MotorType.kBrushless);
        m_topShooterEncoder = m_topShooterMotor.getEncoder();
        m_topShooterMotor.setIdleMode(IdleMode.kCoast);
        m_topShooterMotor.setSmartCurrentLimit(ShooterConstants.kShooterCurrentLimit);

        m_topShooterMotor.burnFlash();
    }

    @Override
    protected void useOutput(double output, double setpoint) {
        m_topShooterMotor.setVoltage(output + m_topShooterFeedforward.calculate(setpoint));
    }

    @Override
    protected double getMeasurement() {
        return (double)m_topShooterEncoder.getVelocity();
    }

    public boolean atSetpoint() {
        return m_controller.atSetpoint();
    }

    public void setVoltage(double p_voltage){
        m_topShooterMotor.setVoltage(p_voltage);
    }

    public void telemetry(){
        // if(firstloop){
        //     SmartDashboard.putNumber("Top shooter commanded velocity", 0);
        //     SmartDashboard.putNumber("Top Shooter P", m_controller.getP());
        //     SmartDashboard.putNumber("Top Shooter I", m_controller.getI());
        //     SmartDashboard.putNumber("Top Shooter D", m_controller.getD());
        //     SmartDashboard.putNumber("Top shooter kS", m_topShooterFeedforward.ks);
        //     SmartDashboard.putNumber("Top shooter kV", m_topShooterFeedforward.kv);  
        //     firstloop = false;  
        // }

        // m_controller.setPID(SmartDashboard.getNumber("Top Shooter P", m_controller.getP()),
        //                     SmartDashboard.getNumber("Top Shooter I", m_controller.getI()),
        //                     SmartDashboard.getNumber("Top Shooter D", m_controller.getD())
        // );

        // m_topShooterFeedforward = new SimpleMotorFeedforward(SmartDashboard.getNumber("Top shooter kS", 0),
        //     SmartDashboard.getNumber("Top shooter kV", 0));

        // setSetpoint(SmartDashboard.getNumber("Top shooter commanded velocity", getSetpoint()));
        // SmartDashboard.putNumber("Top shooter motor velocity", m_topShooterEncoder.getVelocity());
    }

    @Override
    public void periodic(){
        super.periodic();
        telemetry();
    }
}
