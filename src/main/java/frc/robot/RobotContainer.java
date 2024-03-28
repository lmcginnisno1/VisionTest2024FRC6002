// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.GlobalVariables.RobotState;
import frc.robot.GlobalVariables.ScoringMode;
import frc.robot.autos.FiveNoteBlue;
import frc.robot.subsystems.*;
import frc.robot.commands.*;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The driver's controller
  final CommandXboxController m_DriverController = new CommandXboxController(OIConstants.kDriverControllerPort);
  public final XboxController m_DriverControllerHI = new XboxController(OIConstants.kDriverControllerPort);
  final CommandXboxController m_OperatorController = new CommandXboxController(OIConstants.kOperatorControllerPort);
  public final XboxController m_OperatorControllerHI = new XboxController(OIConstants.kDriverControllerPort);
  SendableChooser<Command> autoChooser;

  // The robot's subsystems
  public final GlobalVariables m_variables = new GlobalVariables(m_OperatorControllerHI);//operator controller to rumble at 30 secs.
  public final SUB_Led m_led = new SUB_Led();
  public final SUB_TopShooter m_topShooter = new SUB_TopShooter();
  public final SUB_BottomShooter m_bottomShooter = new SUB_BottomShooter();
  public final SUB_Vision m_vision = new SUB_Vision();
  public final SUB_Intake m_intake = new SUB_Intake(m_variables);
  public final SUB_Arm m_arm = new SUB_Arm(m_variables);
  public final SUB_Drivetrain m_robotDrive = new SUB_Drivetrain(m_variables, m_vision);
  public final SUB_Shooter m_shooter = new SUB_Shooter(m_topShooter, m_bottomShooter, m_led, m_variables);

  

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer(){
    // Configure the button bindings
    m_robotDrive.setDefaultCommand(new CMD_Drive(m_robotDrive, m_DriverController, m_variables));
    m_intake.setDefaultCommand(new CMD_IntakeDefault(this));
    m_led.setDefaultCommand(new CMD_LedDefault(m_led, m_variables));
    autoChooser.setDefaultOption("FiveNoteBlue", new FiveNoteBlue(this));
    SmartDashboard.putData(autoChooser);
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    //main cyle of commands
    m_DriverController.rightBumper().onTrue(new CMD_HandleReadyToShoot(this));

    //when switching to calibration mode, reset odometry to 0,0,0 to calibrate wheel size
    m_DriverController.back().onTrue(new CMD_ToggleCalibrationMode(this));

    //while holding right trigger, chase the closest note in FOV, when released, cancel the command
    m_DriverController.rightTrigger().whileTrue(new CMD_ChaseDownNote(m_robotDrive, m_intake));

    //while holding left trigger, pathfind to our amp, when released, stop
    m_DriverController.rightTrigger().whileTrue(m_robotDrive.pathFindToAmp());

    //while holding b, reverse intake, on release go back to forwards if intaking or off if not
    m_DriverController.b().whileTrue(new CMD_IntakeReverse(m_intake)).onFalse(
      new ConditionalCommand(new CMD_IntakeForward(m_intake), new CMD_IntakeOff(m_intake),
       ()-> m_variables.isRobotState(RobotState.ReadyToIntake)));

    //OPERATOR BINDINGS   

    m_OperatorController.rightBumper().onTrue(new ConditionalCommand(
      new InstantCommand(()-> m_variables.setScoringMode(ScoringMode.AMP)),
      new InstantCommand(()-> m_variables.setScoringMode(ScoringMode.SPEAKER)),
      ()-> m_variables.isScoringMode(ScoringMode.SPEAKER)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
