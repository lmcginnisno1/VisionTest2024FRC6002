// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.*;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  public final GlobalVariables m_Variables = new GlobalVariables();
  public final SUB_Led m_Led = new SUB_Led();
  public final SUB_Climber m_Climber = new SUB_Climber();
  public final SUB_TopShooter m_TopShooter = new SUB_TopShooter();
  public final SUB_BottomShooter m_BottomShooter = new SUB_BottomShooter();
  public final SUB_Vision m_vision = new SUB_Vision();
  public final SUB_Intake m_Intake = new SUB_Intake(m_Led, m_Variables);
  public final SUB_Arm m_arm = new SUB_Arm(m_Variables);
  public final SUB_Drivetrain m_RobotDrive = new SUB_Drivetrain(m_Variables, m_vision);
  public final SUB_Shooter m_Shooter = new SUB_Shooter(m_TopShooter, m_BottomShooter, m_Variables);


  // The driver's controller
  CommandXboxController m_DriverController = new CommandXboxController(OIConstants.kDriverControllerPort);
  XboxController m_DriverControllerHI = new XboxController(OIConstants.kDriverControllerPort);
  CommandXboxController m_OperatorController = new CommandXboxController(OIConstants.kOperatorControllerPort);
  SendableChooser<Command> autoChooser;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer(){
    // Configure the button bindings
    m_RobotDrive.setDefaultCommand(new CMD_Drive(m_RobotDrive, m_DriverController, m_Variables));
    configureButtonBindings();
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData(autoChooser);
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
    m_DriverController.a().onTrue(new ConditionalCommand(new CMD_PrepShot(m_Intake, m_Led, m_Variables),
      new CMD_Shoot(m_Intake, m_Shooter, m_Led, m_Variables), m_Variables::ReadyToShoot));
    m_DriverController.b().onTrue(new CMD_GroundIntakeForward(m_Intake));
    m_DriverController.back().onTrue(new CMD_ToggleCalibrationMode(this));
    m_DriverController.leftTrigger().whileTrue(new CMD_ChaseDownNote(m_RobotDrive, m_Intake));
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
