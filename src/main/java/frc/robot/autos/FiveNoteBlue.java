package frc.robot.autos;


import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.GlobalVariables.ScoringMode;

public class FiveNoteBlue extends SequentialCommandGroup {
  final RobotContainer m_robot;
  final PathPlannerPath m_firstNote = PathPlannerPath.fromPathFile("5NoteBlue1");
  final PathPlannerPath m_secondNote = PathPlannerPath.fromPathFile("5NoteBlue2");
  final PathPlannerPath m_thirdNote = PathPlannerPath.fromPathFile("5NoteBlue3");
  final PathPlannerPath m_fourthNote = PathPlannerPath.fromPathFile("5NoteBlue4");
  final PathPlannerPath m_fourthNoteReturn = PathPlannerPath.fromPathFile("5NoteBlue4Return");
  final PathPlannerPath m_fourthNoteMissed = PathPlannerPath.fromPathFile("5NoteBlue4Missed");
  final PathPlannerPath m_fifthNoteReturn = PathPlannerPath.fromPathFile("5NoteBlue5Return");
  final PathPlannerPath m_fifthNoteMissed = PathPlannerPath.fromPathFile("5NoteBlue5Missed");
  final PathPlannerPath m_sixthNoteReturn = PathPlannerPath.fromPathFile("5NoteBlue6Return");

  public FiveNoteBlue(RobotContainer p_robot) {
    m_robot = p_robot;

    addCommands(
      //ony scoring in speaker during auto
      new InstantCommand(()-> m_robot.m_variables.setScoringMode(ScoringMode.SPEAKER))
      //preloaded note
      ,new CMD_RevShooter(m_robot.m_shooter, m_robot.m_variables)
      ,new CMD_Shoot(m_robot.m_intake, m_robot.m_shooter, m_robot.m_variables)
      ,new CMD_IntakeForward(m_robot.m_intake)
      ,new CMD_ArmSetShoulderGoal(m_robot.m_arm, ArmConstants.kShoulderShootThrough)
      //first note
      ,new CMD_RevShooter(m_robot.m_shooter, m_robot.m_variables)
      ,AutoBuilder.followPath(m_firstNote)
      ,new WaitCommand(1)
      ,new CMD_Shoot(m_robot.m_intake, m_robot.m_shooter, m_robot.m_variables)
      //second note
      ,new CMD_RevShooter(m_robot.m_shooter, m_robot.m_variables)
      ,AutoBuilder.followPath(m_secondNote)
      ,new WaitCommand(1)
      ,new CMD_Shoot(m_robot.m_intake, m_robot.m_shooter, m_robot.m_variables)
      //third note
      ,new CMD_RevShooter(m_robot.m_shooter, m_robot.m_variables)
      ,AutoBuilder.followPath(m_thirdNote)
      ,new WaitCommand(1)
      ,new CMD_Shoot(m_robot.m_intake, m_robot.m_shooter, m_robot.m_variables)
      //fourth note
      ,new CMD_ArmHome(m_robot.m_arm).noWait()
      ,AutoBuilder.followPath(m_fourthNote)
      ,new WaitCommand(1)
      //check if we missed the 4th note if so, try to get the next note instead
      ,new ConditionalCommand(
        //4th note was not taken or missed, run back to the subwoofer
        new SequentialCommandGroup(
          new CMD_RevShooter(m_robot.m_shooter, m_robot.m_variables)
          ,AutoBuilder.followPath(m_fourthNoteReturn)
          ,new CMD_IntakeOff(m_robot.m_intake)
        ),
        //4th note was taken or missed, attempt the 5th note
        new SequentialCommandGroup(
          AutoBuilder.followPath(m_fourthNoteMissed)
          ,new WaitCommand(1)
          //check if we missed the 5th note if so, attempt the 6th note
          ,new ConditionalCommand(
            //5th note was not taken or missed, run back to the subwoofer
            new SequentialCommandGroup(
              new CMD_RevShooter(m_robot.m_shooter, m_robot.m_variables)
              ,AutoBuilder.followPath(m_fifthNoteReturn)
              ,new CMD_IntakeOff(m_robot.m_intake)
            ),
            //5th note was taken, attempt the 6th note
            new SequentialCommandGroup(
              AutoBuilder.followPath(m_fifthNoteMissed)
              ,new WaitCommand(0.5)
              ,new CMD_IntakeOff(m_robot.m_intake)
              ,new CMD_RevShooter(m_robot.m_shooter, m_robot.m_variables)
              ,AutoBuilder.followPath(m_sixthNoteReturn)
            ),
            ()-> m_robot.m_intake.getIntakeSensor()
          )
        ),

        ()-> m_robot.m_intake.getIntakeSensor()
      )
      //shoot the last note
      ,new CMD_Shoot(m_robot.m_intake, m_robot.m_shooter, m_robot.m_variables)
    );
  }
}
