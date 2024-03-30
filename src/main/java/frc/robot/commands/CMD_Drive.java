package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.SUB_Drivetrain;

public class CMD_Drive extends Command{

  private final SUB_Drivetrain m_drivetrain;
  private final CommandXboxController controller;
  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  // private final SlewRateLimiter xspeedLimiter = new SlewRateLimiter(0.5);
  // private final SlewRateLimiter yspeedLimiter = new SlewRateLimiter(0.5);
  // private final SlewRateLimiter rotLimiter = new SlewRateLimiter(0.2);
  public boolean fieldMode = false;
  double deadzone = 0.2;	//variable for amount of deadzone
  double y = 0;           //variable for forward/backward movement
  double x = 0;           //variable for side to side movement
  double turn = 0;        //variable for turning movement

  public CMD_Drive(SUB_Drivetrain m_drivetrain, CommandXboxController m_driverControllerTrigger) {
    this.m_drivetrain = m_drivetrain;
    addRequirements(m_drivetrain);

    this.controller = m_driverControllerTrigger;
  }

  @Override
  public void initialize() {
  }
 
  @Override
  public void execute() {
    // final var xSpeed = xspeedLimiter.calculate(MathUtil.applyDeadband(-controller.getLeftY(), deadzone));
    // final var xSpeed = MathUtil.applyDeadband(-controller.getLeftY(), deadzone);  
    var xSpeed = MathUtil.applyDeadband(-controller.getLeftY(), deadzone);
    xSpeed = Math.pow(xSpeed, 3);
    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    // final var ySpeed = yspeedLimiter.calculate(MathUtil.applyDeadband(-controller.getLeftX(), deadzone));
    // final var ySpeed = MathUtil.applyDeadband(-controller.getLeftX(), deadzone);
    var ySpeed = MathUtil.applyDeadband(-controller.getLeftX(), deadzone);
    ySpeed = Math.pow(ySpeed, 3);
 
    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    

    if(DriverStation.getAlliance().isPresent()){
      if(DriverStation.getAlliance().get() == Alliance.Red){
        xSpeed *= -1;
        ySpeed *= -1;
      }
    }

    // final var rot = rotLimiter.calculate(MathUtil.applyDeadband(-controller.getRightX(), deadzone));
    var rot = MathUtil.applyDeadband(-controller.getRightX(), deadzone);
    rot = Math.pow(rot, 3);

    m_drivetrain.drive(xSpeed, ySpeed, rot, true, true);
  }


  @Override
  public void end(boolean interrupted) {
      m_drivetrain.drive(0.0, 0.0, 0.0, true,false);
  }
}