package frc.robot.commands;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SUB_Drivetrain;
import frc.robot.subsystems.SUB_Intake;

public class CMD_ChaseDownNote extends Command{
    SUB_Drivetrain m_drivetrain;
    PhotonCamera m_backCamera = new PhotonCamera("note_finder_camera");
    SUB_Intake m_intake;
    Timer m_timer = new Timer();
    boolean m_isFinished;

    public CMD_ChaseDownNote(SUB_Drivetrain p_drivetrain, SUB_Intake p_intake){
        m_drivetrain = p_drivetrain;
        m_intake = p_intake;
    }

    @Override
    public void initialize(){
        m_isFinished = false;
        m_intake.setGroundIntakePower(Constants.IntakeConstants.kIntakeForward);
        m_intake.setIndexerPower(Constants.IntakeConstants.kIndexerForward);
        m_timer.reset();
        m_timer.start();
    }

    @Override
    public void execute(){
        double target_yaw = m_backCamera.getLatestResult().getBestTarget().getYaw();
        double target_area = m_backCamera.getLatestResult().getBestTarget().getArea();
        double heading_error = m_drivetrain.getPose().getRotation().getDegrees() - (180 + target_yaw);

        double rot = heading_error * 0.002;
        double xSpeed = target_area / (Math.pow(target_area, 3));
        MathUtil.clamp(xSpeed, 0.1, 0.75);

        m_drivetrain.drive(xSpeed, 0, rot, false, true);

        if(m_intake.getIntakeSensor() || m_timer.get() > 10){
            m_intake.setGroundIntakePower(Constants.IntakeConstants.kIntakeOff);
            m_intake.setIndexerPower(Constants.IntakeConstants.kIndexerOff);
            m_drivetrain.drive(0, 0, 0, false, true);
            m_isFinished = true;
        }
    }

    @Override
    public boolean isFinished(){
        return m_isFinished;
    }
}
