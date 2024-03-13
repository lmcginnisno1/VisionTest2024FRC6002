package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.GlobalVariables;
import frc.robot.GlobalVariables.SourceSlot;
import frc.robot.subsystems.SUB_Drivetrain;

public class CMD_AutoDriveToSource extends Command{
    final SUB_Drivetrain m_drivetrain;
    final GlobalVariables m_variables;
    PathPlannerPath m_path;
    Command m_pathFindingCommand;
    DriverStation.Alliance m_Alliance;
    public CMD_AutoDriveToSource(SUB_Drivetrain p_drivetrain, GlobalVariables p_variables){
        m_drivetrain = p_drivetrain;
        m_variables = p_variables;
    }

    @Override
    public void initialize(){
        if(DriverStation.getAlliance().isPresent()){
            m_Alliance = DriverStation.getAlliance().get();
        }

        Pose2d m_robotPose = m_drivetrain.getPose();
        //find the closest source slot to us base on X, using blue alliance coordinate system
        if(m_Alliance == DriverStation.Alliance.Blue){
            if(m_robotPose.getX() < 15.5){
                m_variables.SetSourceSlot(SourceSlot.LEFT);
            }else if(m_robotPose.getX() < 16){
                m_variables.SetSourceSlot(SourceSlot.MIDDLE);
            }else if(m_robotPose.getX() <= 16.5){
                m_variables.SetSourceSlot(SourceSlot.RIGHT);
            }else{
                return;
            }
        }else if (m_Alliance == DriverStation.Alliance.Blue){
           if(m_robotPose.getX() > 2.5){
                m_variables.SetSourceSlot(SourceSlot.RIGHT);
            }else if(m_robotPose.getX() > 2){
                m_variables.SetSourceSlot(SourceSlot.MIDDLE);
            }else if(m_robotPose.getX() > 1.1){
                m_variables.SetSourceSlot(SourceSlot.LEFT);
            }else{
                return;
            } 
        }

        // Load the path we want to pathfind to and follow
        if(m_variables.getSelectedSourceSlot() == SourceSlot.LEFT){
            m_path = PathPlannerPath.fromPathFile("source slot left");
        }else if(m_variables.getSelectedSourceSlot() == SourceSlot.MIDDLE){
            m_path = PathPlannerPath.fromPathFile("source slot middle");
        }else if(m_variables.getSelectedSourceSlot() == SourceSlot.RIGHT){
            m_path = PathPlannerPath.fromPathFile("source slot right");
        }else{
            return;
        }

        if(DriverStation.getAlliance().isPresent()){
            if(m_Alliance == Alliance.Red){
                m_path.flipPath();
            }
        }else{
            return;
        }

        // Create the constraints to use while pathfinding. The constraints defined in the path will only be used for the path.
        PathConstraints constraints = new PathConstraints(
                4.0, 4.0,
                Units.degreesToRadians(180), Units.degreesToRadians(180));

        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        m_pathFindingCommand = AutoBuilder.pathfindThenFollowPath(
                m_path,
                constraints,
                3.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
        );

        m_pathFindingCommand.schedule();
    }

    @Override
    public boolean isFinished(){
        return m_pathFindingCommand.isFinished();
    }
}
