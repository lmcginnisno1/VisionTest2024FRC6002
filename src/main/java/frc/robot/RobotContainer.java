// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.*;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
  public final GlobalVariables m_variables = new GlobalVariables();
  public final SUB_Led m_led = new SUB_Led();
  public final SUB_Climber m_climber = new SUB_Climber();
  public final SUB_TopShooter m_topShooter = new SUB_TopShooter();
  public final SUB_BottomShooter m_bottomShooter = new SUB_BottomShooter();
  public final SUB_Vision m_vision = new SUB_Vision();
  public final SUB_Intake m_intake = new SUB_Intake(m_led, m_variables);
  public final SUB_Arm m_arm = new SUB_Arm(m_variables);
  public final SUB_Drivetrain m_robotDrive = new SUB_Drivetrain(m_variables, m_vision);
  public final SUB_Shooter m_shooter = new SUB_Shooter(m_topShooter, m_bottomShooter, m_led, m_variables);

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
    NamedCommands.registerCommand("Ready", new CMD_RevShooter(m_shooter, m_variables));
    NamedCommands.registerCommand("Aim drive", new CMD_AutoAim(m_robotDrive, m_variables));
    NamedCommands.registerCommand("Aim arm", new CMD_ArmAim(m_arm, m_variables));
    NamedCommands.registerCommand("Aim arm noWait", new CMD_ArmAim(m_arm, m_variables).noWait());
    NamedCommands.registerCommand("Auto Aim", new SequentialCommandGroup(
      new CMD_ArmAim(m_arm, m_variables).noWait(),
      new CMD_AutoAim(m_robotDrive, m_variables)
    ));
    NamedCommands.registerCommand("Fire", new CMD_Shoot(m_intake, m_shooter, m_variables));
    NamedCommands.registerCommand("Home Arm", new CMD_ArmHome(m_arm));
    NamedCommands.registerCommand("Home Arm noWait", new CMD_ArmHome(m_arm).noWait());

    m_robotDrive.setDefaultCommand(new CMD_Drive(m_robotDrive, m_DriverController, m_variables));
    autoChooser = AutoBuilder.buildAutoChooser();
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
    m_DriverController.a().onTrue(new ConditionalCommand(new CMD_PrepShot(m_intake, m_led, m_variables),
      new CMD_Shoot(m_intake, m_shooter, m_variables), m_variables::ReadyToShoot));
    m_DriverController.b().onTrue(new CMD_GroundIntakeForward(m_intake));
    m_DriverController.back().onTrue(new CMD_ToggleCalibrationMode(this));
    m_DriverController.leftTrigger().whileTrue(new CMD_ChaseDownNote(m_robotDrive, m_intake));
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
