package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.SUB_Drivetrain;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

public class BackAndForth extends SequentialCommandGroup{
    SUB_Drivetrain m_drivetrain;
    public BackAndForth(SUB_Drivetrain p_drivetrain){
        m_drivetrain = p_drivetrain;
        addCommands(
            m_BackAndForth()
        );
    }

    SequentialCommandGroup m_BackAndForth(){
        SequentialCommandGroup cmds = new SequentialCommandGroup();
        PathPlannerPath Forward = PathPlannerPath.fromPathFile("forward");
        PathPlannerPath Backward = PathPlannerPath.fromPathFile("backward");
        cmds.addCommands(
            followPath(Forward),
            followPath(Backward),
            repeatedly()
        );
        return cmds;
    }

    public Command followPath(PathPlannerPath path){
        return AutoBuilder.followPath(path);
    }
}
