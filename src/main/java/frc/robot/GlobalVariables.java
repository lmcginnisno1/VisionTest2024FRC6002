package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GlobalVariables extends SubsystemBase{

    private Pose2d m_robotPose = new Pose2d(0.0, 0.0, new Rotation2d(0.0));
    private Pose2d m_limelightPose2d = new Pose2d(0.0, 0.0, new Rotation2d(0.0));
    private double m_robotVelocity = 0.0;
    public double m_AngleToTarget = 0;

    public enum SourceSlot{
        LEFT,
        MIDDLE,
        RIGHT
    }

    private SourceSlot m_SourceSlotSelected = SourceSlot.MIDDLE;

    @Override
    public void periodic(){
        
    }

    public void setRobotPose(Pose2d p_robotPose){
        m_robotPose = p_robotPose;
    }

    public Pose2d getRobotPose(){
        return m_robotPose;
    }

    public void setRobotVelocity(double p_robotVelocity){
        m_robotVelocity = p_robotVelocity;
    }

    public double getRobotVelcoity(){
        return m_robotVelocity;
    }

    public void setLimelightPose2d(Pose2d p_limelightPose2d){
        m_limelightPose2d = p_limelightPose2d;
    }

    public Pose2d getLimelightPose2d(){
        return m_limelightPose2d;
    }

    public void setAngleToTarget(double p_angle){
        m_AngleToTarget = p_angle;
    }

    public double getAngleToTarget(){
        return m_AngleToTarget;
    }

    public void SetSourceSlot(SourceSlot p_SourceSlot){
        m_SourceSlotSelected = p_SourceSlot;
    }

    public SourceSlot getSelectedSourceSlot(){
        return m_SourceSlotSelected;
    }
}
