package frc.robot.commands;

import frc.robot.subsystems.SUB_Arm;
import frc.robot.subsystems.SUB_Intake;
import frc.robot.subsystems.SUB_Shooter;

public class CMD_PrepShot {
    final SUB_Shooter m_shooter;
    final SUB_Intake m_intake;
    final SUB_Arm m_arm;
    
    public CMD_PrepShot(SUB_Shooter p_shooter, SUB_Intake p_intake, SUB_Arm p_arm){
        m_shooter = p_shooter;
        m_intake = p_intake;
        m_arm = p_arm;
    }
}
