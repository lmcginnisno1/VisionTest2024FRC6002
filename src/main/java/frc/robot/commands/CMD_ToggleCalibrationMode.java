package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class CMD_ToggleCalibrationMode extends Command{
    final RobotContainer m_RobotContainer;

    public CMD_ToggleCalibrationMode(RobotContainer p_robotContainer){
        m_RobotContainer = p_robotContainer;
    }

    @Override 
    public void initialize(){
        if(!m_RobotContainer.m_variables.getCalibrationMode()){
            m_RobotContainer.m_variables.setCalibrationMode(true);
            m_RobotContainer.m_robotDrive.resetPose(new Pose2d(0, 0, new Rotation2d(Units.degreesToRadians(0))));
        }else{
            m_RobotContainer.m_variables.setCalibrationMode(false);
        }
    }
}
