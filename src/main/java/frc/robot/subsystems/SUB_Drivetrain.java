// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.WPIUtilJNI;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.GlobalVariables;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;
import frc.utils.SwerveUtils;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;


public class SUB_Drivetrain extends SubsystemBase {
  // Create MAXSwerveModules
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset);

  // The gyro sensor
  private final AHRS m_navx = new AHRS(Port.kMXP);

  // Slew rate filter variables for controlling lateral acceleration
  private double m_currentRotation = 0.0;
  private double m_currentTranslationDir = 0.0;
  private double m_currentTranslationMag = 0.0;

  private SlewRateLimiter m_magLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);
  private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewRate);
  private double m_prevTime = WPIUtilJNI.now() * 1e-6;
  ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0, 0, 0);
  GlobalVariables m_variables;
  SUB_Vision m_vision;
  double m_distanceToTarget = 0;
  double m_AngleToTarget = 0;
  // Odometry class for tracking robot pose
  SwerveDrivePoseEstimator m_odometry = 
    new SwerveDrivePoseEstimator(DriveConstants.kDriveKinematics, getHeadingRotation2d(), getModulePositions(), getPose(),
    VisionConstants.stateStdDevs, VisionConstants.visionStdDevs);

  /** Creates a new DriveSubsystem. */
  public SUB_Drivetrain(GlobalVariables p_variables, SUB_Vision p_vision) {
    m_variables = p_variables;
    m_vision = p_vision;

    AutoBuilder.configureHolonomic(
      this::getPose, // Robot pose supplier
      this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
      this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
      this::drive, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
      Constants.DriveConstants.kHolonomicPathFollowerConfig,
      () -> false,
      this // Reference to this subsystem to set requirements
    );
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
      Rotation2d.fromDegrees(m_navx.getAngle()),
      getModulePositions());

      var visionEst = m_vision.getEstimatedGlobalPose();
        visionEst.ifPresent(
                est -> {
                    var estPose = est.estimatedPose.toPose2d();
                    // Change our trust in the measurement based on the tags we can see
                    var estStdDevs = m_vision.getEstimationStdDevs(estPose);

                    m_odometry.addVisionMeasurement(
                            est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
                });

      SmartDashboard.putNumber("robot X", getPose().getX());
      SmartDashboard.putNumber("robot Y", getPose().getY());
      SmartDashboard.putNumber("robot Heading", getPose().getRotation().getDegrees());

     if(visionEst.isPresent()){
      SmartDashboard.putNumber("visionX", visionEst.get().estimatedPose.getX());
      SmartDashboard.putNumber("visionY", visionEst.get().estimatedPose.getY());
      SmartDashboard.putNumber("visionHeading", Units.radiansToDegrees(visionEst.get().estimatedPose.getRotation().getAngle()));
     }

      if(DriverStation.getAlliance().isPresent()){
        if(DriverStation.getAlliance().get() == Alliance.Blue){

          m_distanceToTarget = Math.sqrt(Math.pow(0 - getPose().getX(), 2) + Math.pow(5.5 - getPose().getY(), 2));
          SmartDashboard.putNumber("distance to target", m_distanceToTarget);

          m_AngleToTarget = Math.asin((5.5 - getPose().getY()) / m_distanceToTarget);
          SmartDashboard.putNumber("calculated angle", Math.round(Units.radiansToDegrees(m_AngleToTarget)));

        }else if(DriverStation.getAlliance().get() == Alliance.Red){

          m_distanceToTarget = Math.sqrt(Math.pow(16.5 - getPose().getX(), 2) + Math.pow(5.5 - getPose().getY(), 2));
          SmartDashboard.putNumber("distance to target", m_distanceToTarget);

          m_AngleToTarget = Math.asin((5.5 - getPose().getY()) / m_distanceToTarget);
          SmartDashboard.putNumber("calculated angle", Math.round(Units.radiansToDegrees(m_AngleToTarget)));
        }

      }
      
      telemetry();
  }

  public void telemetry(){
    // SmartDashboard.putNumber("front left P", m_frontLeft.getDrivingP());
    // SmartDashboard.putNumber("front left D", m_frontLeft.getDrivingD());
    // SmartDashboard.putNumber("front left F", m_frontLeft.getDrivingF());
    // m_frontLeft.setDrivingP(SmartDashboard.getNumber("front left P", 0));
    // m_frontLeft.setDrivingD(SmartDashboard.getNumber("front left D", 0));
    // m_frontLeft.setDrivingF(SmartDashboard.getNumber("front left F", 0));

    // SmartDashboard.putNumber("front right P", m_frontRight.getDrivingP());
    // SmartDashboard.putNumber("front right D", m_frontRight.getDrivingD());
    // SmartDashboard.putNumber("front right F", m_frontRight.getDrivingF());
    // m_frontRight.setDrivingP(SmartDashboard.getNumber("front right P", 0));
    // m_frontRight.setDrivingD(SmartDashboard.getNumber("front right D", 0));
    // m_frontRight.setDrivingF(SmartDashboard.getNumber("front right F", 0));

    // SmartDashboard.putNumber("back left P", m_rearLeft.getDrivingP());
    // SmartDashboard.putNumber("back left D", m_rearLeft.getDrivingD());
    // SmartDashboard.putNumber("back left F", m_rearLeft.getDrivingF());
    // m_rearLeft.setDrivingP(SmartDashboard.getNumber("back left P", 0));
    // m_rearLeft.setDrivingD(SmartDashboard.getNumber("back left D", 0));
    // m_rearLeft.setDrivingF(SmartDashboard.getNumber("back left F", 0));

    // SmartDashboard.putNumber("back right P", m_rearRight.getDrivingP());
    // SmartDashboard.putNumber("back right D", m_rearRight.getDrivingD());
    // SmartDashboard.putNumber("back right F", m_rearRight.getDrivingF());
    // m_rearRight.setDrivingP(SmartDashboard.getNumber("back right P", 0));
    // m_rearRight.setDrivingD(SmartDashboard.getNumber("back right D", 0));
    // m_rearRight.setDrivingF(SmartDashboard.getNumber("back right F", 0));

    // SmartDashboard.putNumber("front left desired velocity", m_frontLeft.getDesiredVelocity());
    // SmartDashboard.putNumber("front left velocity", m_frontLeft.getVelocity());

    // SmartDashboard.putNumber("front right desired velocity", m_frontRight.getDesiredVelocity());
    // SmartDashboard.putNumber("front right velocity", m_frontRight.getVelocity());

    // SmartDashboard.putNumber("back left desired velocity", m_rearLeft.getDesiredVelocity());
    // SmartDashboard.putNumber("back left velocity", m_rearLeft.getVelocity());

    // SmartDashboard.putNumber("back right desired velocity", m_rearRight.getDesiredVelocity());
    // SmartDashboard.putNumber("back right velocity", m_rearRight.getVelocity());
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getEstimatedPosition();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetPose(Pose2d pose) {
    m_odometry.resetPosition(
        Rotation2d.fromDegrees(m_navx.getAngle()),
        getModulePositions(),
        pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   * @param rateLimit     Whether to enable rate limiting for smoother control.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {
    
    double xSpeedCommanded;
    double ySpeedCommanded;

    if (rateLimit) {
      // Convert XY to polar for rate limiting
      double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
      double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

      // Calculate the direction slew rate based on an estimate of the lateral acceleration
      double directionSlewRate;
      if (m_currentTranslationMag != 0.0) {
        directionSlewRate = Math.abs(DriveConstants.kDirectionSlewRate / m_currentTranslationMag);
      } else {
        directionSlewRate = 500.0; //some high number that means the slew rate is effectively instantaneous
      }
      

      double currentTime = WPIUtilJNI.now() * 1e-6;
      double elapsedTime = currentTime - m_prevTime;
      double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, m_currentTranslationDir);
      if (angleDif < 0.45*Math.PI) {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
      }
      else if (angleDif > 0.85*Math.PI) {
        if (m_currentTranslationMag > 1e-4) { //some small number to avoid floating-point errors with equality checking
          // keep currentTranslationDir unchanged
          m_currentTranslationMag = m_magLimiter.calculate(0.0);
        }
        else {
          m_currentTranslationDir = SwerveUtils.WrapAngle(m_currentTranslationDir + Math.PI);
          m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
        }
      }
      else {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(0.0);
      }
      m_prevTime = currentTime;
      
      xSpeedCommanded = m_currentTranslationMag * Math.cos(m_currentTranslationDir);
      ySpeedCommanded = m_currentTranslationMag * Math.sin(m_currentTranslationDir);
      m_currentRotation = m_rotLimiter.calculate(rot);


    } else {
      xSpeedCommanded = xSpeed;
      ySpeedCommanded = ySpeed;
      m_currentRotation = rot;
    }

    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = m_currentRotation * DriveConstants.kMaxAngularSpeed;

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, Rotation2d.fromDegrees(m_navx.getAngle()))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));

    m_chassisSpeeds = fieldRelative
      ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, Rotation2d.fromDegrees(m_navx.getAngle()))
      : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered);
    
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  public void drive(ChassisSpeeds p_chassisSpeeds){
    var swerveModuleStates =  DriveConstants.kDriveKinematics.toSwerveModuleStates(p_chassisSpeeds);

    SwerveDriveKinematics.desaturateWheelSpeeds(
      swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);

    m_chassisSpeeds = p_chassisSpeeds;

    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  public ChassisSpeeds getChassisSpeeds(){
    return DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
  }
  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  public SwerveModulePosition[] getModulePositions(){
    SwerveModulePosition[] modulePositions = {m_frontLeft.getPosition(), m_frontRight.getPosition(),
                                           m_rearLeft.getPosition(), m_rearRight.getPosition()};
    return modulePositions;
  }

  public SwerveModuleState[] getModuleStates(){
    SwerveModuleState[] moduleStates = {m_frontLeft.getState(), m_frontRight.getState(),
                                           m_rearLeft.getState(), m_rearRight.getState()};
    return moduleStates;
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_navx.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return Rotation2d.fromDegrees(m_navx.getAngle()).getDegrees();
  }

  public Rotation2d getHeadingRotation2d() {
    return Rotation2d.fromDegrees(m_navx.getAngle());
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_navx.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }
}
