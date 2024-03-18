package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GlobalVariables extends SubsystemBase{

    private Pose2d m_robotPose = new Pose2d(0.0, 0.0, new Rotation2d(0.0));
    private double m_AngleToTarget = 0;
    private double m_DistanceToTarget = 0;
    boolean m_readyToShoot = false;
    boolean m_calibrationMode = false;

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
