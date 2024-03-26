package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ClimberConstants;

public class SUB_Climber extends SubsystemBase{

    final PWM m_climberServo = new PWM(ClimberConstants.kClimberServoPWMPort);

    public SUB_Climber(){}

    public void setOpen(){
        m_climberServo.setPosition(ClimberConstants.kClimberServoUnhooked);
    }

    public void setClosed(){
        m_climberServo.setPosition(ClimberConstants.kClimberServoHooked);
    }
}