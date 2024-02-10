package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.GlobalVariables;
import frc.robot.subsystems.SUB_Arm;
import frc.robot.subsystems.SUB_BottomShooter;

public class CMD_BottomShooterDefault extends Command {
  SUB_BottomShooter m_BottomShooter;
  SUB_Arm m_Arm;
  GlobalVariables m_Variables;
  Boolean m_idleMode;

  public CMD_BottomShooterDefault(SUB_BottomShooter p_BottomShooter, SUB_Arm p_Arm, GlobalVariables p_variables) {
    m_BottomShooter = p_BottomShooter;
    m_Arm = p_Arm;
    m_Variables = p_variables;
    addRequirements(m_BottomShooter);
  }

  @Override
  public void initialize(){
  }

  @Override
  public void execute(){
    double robotX = m_Variables.getRobotPose().getX();

    if(DriverStation.getAlliance().get() == DriverStation.Alliance.Blue && Units.metersToFeet(robotX) < 27.5){
      /* detect if we are on blue alliace and if we are on our side of the field,
      if so calculate BottomShooter velocity needed to reach the speaker from current robot pose*/ 
      m_BottomShooter.setSetpoint(Units.metersToInches(robotX) * Constants.ShooterConstants.kBottomShooterVelocityFF);
      //RPM needed to reach target from current 
      m_idleMode = false;
      return;
    }else if(DriverStation.getAlliance().get() == DriverStation.Alliance.Red && Units.metersToFeet(robotX) < 27.5){
      m_BottomShooter.setSetpoint((Units.metersToInches(robotX)) * Constants.ShooterConstants.kBottomShooterVelocityFF);
      //RPM needed to reach target from current pose
      m_idleMode = false;
      return;
    }else if(m_idleMode == false){
      m_BottomShooter.setSetpoint(100);
      m_idleMode = true;
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
