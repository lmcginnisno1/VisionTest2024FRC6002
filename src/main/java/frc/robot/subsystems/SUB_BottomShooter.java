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

public class SUB_BottomShooter extends PIDSubsystem{
    final CANSparkMax m_bottomShooterMotorMain;
    final CANSparkMax m_bottomShooterMotorFollower;
    final RelativeEncoder m_bottomShooterEncoder;
    double setpoint;

    private SimpleMotorFeedforward m_bottomShooterFeedforward =
      new SimpleMotorFeedforward(
          ShooterConstants.kSBottom, ShooterConstants.kVBottom);

    public SUB_BottomShooter(){
        super(new PIDController(ShooterConstants.kBottomShooterP, ShooterConstants.kBottomShooterI, ShooterConstants.kBottomShooterD));
        getController().setTolerance(ShooterConstants.kShooterToleranceRPS);
        setSetpoint(0);
        m_bottomShooterMotorMain = new CANSparkMax(ShooterConstants.kBottomShooterMotorMainCANId, MotorType.kBrushless);
        m_bottomShooterEncoder = m_bottomShooterMotorMain.getEncoder();
        m_bottomShooterMotorMain.setIdleMode(IdleMode.kCoast);
        m_bottomShooterMotorMain.setSmartCurrentLimit(ShooterConstants.kShooterCurrentLimit);

        m_bottomShooterMotorFollower = new CANSparkMax(ShooterConstants.kBottomShooterMotorFollowerCANId, MotorType.kBrushless);
        m_bottomShooterMotorFollower.setSmartCurrentLimit(ShooterConstants.kShooterCurrentLimit);
        m_bottomShooterMotorFollower.setIdleMode(IdleMode.kCoast);  
        m_bottomShooterMotorFollower.follow(m_bottomShooterMotorMain);

        m_bottomShooterMotorMain.burnFlash();
        m_bottomShooterMotorFollower.burnFlash();
        setSetpoint(0);
        m_bottomShooterEncoder.setPosition(0);
    }

    @Override
    protected void useOutput(double output, double setpoint) {
        m_bottomShooterMotorMain.setVoltage(output + m_bottomShooterFeedforward.calculate(setpoint));
    }

    @Override
    protected double getMeasurement() {
        return (double)m_bottomShooterEncoder.getVelocity();
    }

    public void setVoltage(double p_voltage){
        m_bottomShooterMotorMain.setVoltage(p_voltage);
    }

    public boolean atSetpoint() {
        return m_controller.atSetpoint();
    }

    public void telemetry(){
        SmartDashboard.putNumber("Bottom shooter motor velocity", m_bottomShooterEncoder.getVelocity());
        SmartDashboard.putNumber("Bottom shooter commanded velocity", 0);
        setSetpoint(SmartDashboard.getNumber("Bottom shooter commanded velocity", getSetpoint()));
        SmartDashboard.putNumber("Bottom Shooter P", m_controller.getP());
        SmartDashboard.putNumber("Bottom Shooter I", m_controller.getI());
        SmartDashboard.putNumber("Bottom Shooter D", m_controller.getD());
        SmartDashboard.putNumber("Bottom shooter kS", m_bottomShooterFeedforward.ks);
        SmartDashboard.putNumber("Bottom shooter kV", m_bottomShooterFeedforward.kv);
        
        m_controller.setPID(SmartDashboard.getNumber("Bottom Shooter P", m_controller.getP()),
                            SmartDashboard.getNumber("Bottom Shooter I", m_controller.getI()),
                            SmartDashboard.getNumber("Bottom Shooter D", m_controller.getD())
        );

        m_bottomShooterFeedforward = new SimpleMotorFeedforward(SmartDashboard.getNumber("Bottom shooter kS", 0),
            SmartDashboard.getNumber("Bottom shooter kV", 0));
    }

    @Override
    public void periodic(){
        // super.periodic();
        setpoint += 0.001;
        m_bottomShooterMotorMain.setVoltage(setpoint);
        if(m_bottomShooterEncoder.getPosition() > 0){
            SmartDashboard.putNumber("bottom shooter kS", setpoint - 0.001);
            setpoint = 0;
            m_bottomShooterMotorMain.setVoltage(setpoint);
        }
        telemetry();
    }
}
