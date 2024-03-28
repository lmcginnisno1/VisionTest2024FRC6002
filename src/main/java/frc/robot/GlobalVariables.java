package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class GlobalVariables extends SubsystemBase{

    private Pose2d m_robotPose = new Pose2d(0.0, 0.0, new Rotation2d(0.0));
    private double m_AngleToTarget = 0;
    private double m_DistanceToTarget = 0;
    boolean m_readyToShoot = false;
    boolean m_calibrationMode = false;
    final Timer m_matchTimer = new Timer();
    int m_timeRemaining = 0;
    final XboxController m_operatorController;

    public GlobalVariables(XboxController p_operatorController){
        m_operatorController = p_operatorController;
    }

    public enum RobotState{
        Home,
        ReadyToIntake,
        Stow,
        ReadyToShoot,
        TransitioningToHome,
        TransitioningToShoot,
        Climbing,
        Hanging
    }

    public enum ScoringMode{
        SPEAKER,
        AMP
    }

    private ScoringMode m_scoringMode = ScoringMode.SPEAKER;

    private RobotState m_robotState = RobotState.Home;

    public void setRobotState(RobotState p_robotState){
        m_robotState = p_robotState;
    }

    public boolean isRobotState(RobotState p_RobotState){
        return m_robotState == p_RobotState;
    }

    public void setScoringMode(ScoringMode p_scoringMode){
        m_scoringMode = p_scoringMode;
    }

    public boolean isScoringMode(ScoringMode p_scoringMode){
        return m_scoringMode == p_scoringMode;
    }

    @Override
    public void periodic(){
        if(DriverStation.isEnabled()){
            m_matchTimer.start();
        }else{
            m_matchTimer.reset();
        }
        if(DriverStation.isAutonomous()){
            m_timeRemaining = 15 - (int)m_matchTimer.get();
        }
        if(DriverStation.isTeleop()){
            m_timeRemaining = 135 - (int)m_matchTimer.get();
            //rumble operator controller at 30 seconds remaining
            if(m_timeRemaining == 30){
                new SequentialCommandGroup(
                    new InstantCommand(()-> m_operatorController.setRumble(RumbleType.kBothRumble, 1))
                    ,new WaitCommand(1)
                    ,new InstantCommand(()-> m_operatorController.setRumble(RumbleType.kBothRumble, 0))
                ).schedule();
            }
        }
        SmartDashboard.putNumber("Match Time", m_timeRemaining);
    }

    public void setRobotPose(Pose2d p_robotPose){
        m_robotPose = p_robotPose;
    }

    public Pose2d getRobotPose(){
        return m_robotPose;
    }

    public void setAngleToTarget(double p_angle){
        m_AngleToTarget = p_angle;
    }

    public double getAngleToTarget(){
        return m_AngleToTarget;
    }

    public void setDistanceToTarget(double p_distance){
        m_DistanceToTarget = p_distance;
    }

    public double getDistanceToTarget(){
        return m_DistanceToTarget;
    }

    public void setReadyToShoot(boolean p_readyToShoot){
        m_readyToShoot = p_readyToShoot;
    }

    public boolean ReadyToShoot(){
        return m_readyToShoot;
    }

    public void setCalibrationMode(boolean p_calibrating){
        m_calibrationMode = p_calibrating;
    }

    public boolean getCalibrationMode(){
        return m_calibrationMode;
    }
}
