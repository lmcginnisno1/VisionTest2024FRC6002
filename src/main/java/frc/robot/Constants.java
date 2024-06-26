// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.utils.LinearInterpolator;
import frc.utils.led.Color;
import frc.utils.led.pattern.*;

import com.pathplanner.lib.util.PIDConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 5.6;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    public static final double kDirectionSlewRate = 1.2; // radians per second
    public static final double kMagnitudeSlewRate = 1.8; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(18.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(25.5);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 2;
    public static final int kFrontLeftTurningCanId = 1;
    public static final int kFrontRightDrivingCanId = 4;
    public static final int kFrontRightTurningCanId = 3;
    public static final int kRearLeftDrivingCanId = 6;
    public static final int kRearLeftTurningCanId = 5;
    public static final int kRearRightDrivingCanId = 8;
    public static final int kRearRightTurningCanId = 7;

    public static final HolonomicPathFollowerConfig kHolonomicPathFollowerConfig = new HolonomicPathFollowerConfig(
      new PIDConstants(1.0, 0.0, 0.0), // Translation PID constants
      new PIDConstants(1.0, 0.0, 0.0), // Rotation PID constants
      kMaxSpeedMetersPerSecond,
      0.4, // Drive base radius in meters. Distance from robot center to furthest module.
      new ReplanningConfig() // Default path replanning config. See the API for the options here
    );

    public static final boolean kGyroReversed = false;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth will result in a
    // robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 15;

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 20) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;

    public static final double kXFactor = .9714;  // if actual is smaller than odo go down  

    public static final double kDrivingEncoderPositionFactor = ((kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction) * kXFactor; // meters
    public static final double kDrivingEncoderVelocityFactor = (((kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction) / 60.0) * kXFactor; // meters per second

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

    public static final double kDrivingP = 0.17;//0.04
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0;
    public static final double kDrivingFF = 0.21; //1 / kDriveWheelFreeSpeedRps
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final double kDrivingA = .4;
    public static final double kDrivingS = 0.17;
    public static final double kDrivingV = 2.28;

    public static final double kTurningP = 1.5;
    public static final double kTurningI = 0.0;
    public static final double kTurningD = 0;
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public static final int kDrivingMotorCurrentLimit = 50; // amps
    public static final int kTurningMotorCurrentLimit = 20; // amps
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    public static final double kDriveDeadband = 0.05;
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  public static final class IntakeConstants {
    public static final int kMainGroundIntakeMotorCANId = 11;
    public static final int kFollowerGroundIntakeMotorCANId = 12;
    public static final int kIndexerMotorCANId = 13;
    public static final int kIndexerSensorDigitalPort = 9;

    public static final double kGroundIntakeP = 0.0;
    public static final double kGroundIntakeI = 0;
    public static final double kGroundIntakeD = 0.0;
    public static final double kGroundIntakeFF = 0.00018;

    public static final double kIntakeForward = 3000;
    public static final double kIntakeOff = 0.0;
    public static final double kIntakeReverse = -3000;

    public static final double kIndexerP = 0.0;
    public static final double kIndexerI = 0.0;
    public static final double kIndexerD = 0.0;
    public static final double kIndexerFF = 0.00024;

    public static final double kIndexerForward = 2000;
    public static final double kIndexerOff = 0.0;
    public static final double kIndexerReverse = -2000;

    public static final int kIndexerCurrentLimit = 40;
    public static final int kGroundIntakeCurrentLimit = 40;
  }

  public static final class ShooterConstants{
    public static final int kShooterCurrentLimit = 40;

    public static final int kBottomShooterMotorFollowerCANId = 21;
    public static final int kBottomShooterMotorMainCANId = 22;
    public static final int kTopShooterMotorCANId = 23;

    public static final double kShooterOff = 0.0;
    public static final double kShooterAmp = 2000;
    public static final double kShooterSubwoofer = 2000;

    public static final int kShooterToleranceRPS = 100;

    public static final double kTopShooterP = 0.0075;
    public static final double kTopShooterI = 0.0;
    public static final double kTopShooterD = 0.001;
    public static final double kSTop = 0.22;
    public static final double kVTop = 0.0022;

    public static final double kBottomShooterP = 0.0075;
    public static final double kBottomShooterI = 0.0;
    public static final double kBottomShooterD = 0.001;
    public static final double kSBottom = 0.24;
    public static final double kVBottom = 0.0022;

    public static final double kShooterMaxAcceleration = 4500;
    public static final double kShooterMaxVelocity = 4500;
    public static final double kShooterLongshotSpeed = 3500;
    public static final double kShooterMinOupt = -1.0;
    public static final double kShooterMaxOutput = 1.0;
  }

  public static final class ArmConstants{
    public static final int kShoulderMotorMainCANId = 9;
    public static final int kShoulderFollowerCANId = 10;
    public static final int kElbowMotorCANId = 14;

    public static final int kShoulderCurrentLimit = 50;
    public static final int kElbowCurrentLimit = 50;

    public static final double kShoulderP = 3.5;
    public static final double kShoulderI = 0;
    public static final double kShoulderD = 0;
    public static final double kShoulderFF = 0.0;

    public static final double kShoulderkS = 0.3;
    public static final double kShoulderkG = 0.275;
    public static final double kShoulderkV = 0.0;

    public static final double kShoulderMaxVelocityRadPerSecond = 6.1;
    public static final double kShoulderMaxAccelerationRadPerSecSquared = 8.1;

    public static final double kShoulderPositionConversionFactor = Math.PI * 2; // radians
    public static final double kShoulderVelocityConversionFactor = kShoulderPositionConversionFactor / 60; // radians per second

    public static final double kElbowP = 2;
    public static final double kElbowI = 0;
    public static final double kElbowD = 0.0;
    public static final double kElbowFF = 0.0;
    
    public static final double kElbowkS = 0.3;
    public static final double kElbowkG = 0.1;
    public static final double kElbowkV = 0.0;

    public static final double kElbowMaxVelocityRadPerSecond = 6.1;
    public static final double kElbowMaxAccelerationRadPerSecSquared = 8.1;

    public static final double kElbowPositionConversionFactor = (Math.PI * 2) / 3; // 3:1 ratio to shaft
    public static final double kElbowVelocityConversionFactor = kElbowPositionConversionFactor / 60;

    public static final double kShoulderHome = -45;
    public static final double kShoulderShootThrough = -40;//used in auto to fire notes quicker
    public static final double kShoulderAmp = 30;
    public static final double kElbowHome = 10; 
    public static final double kElbowAmp = 18.5; 
    public static final double kElbowGroundIntake = 45;

    public static final double kTolerance = 1;

    public static final double[][] kShoulderInterpolatorValues = {
      {36, -45},
      {50, -45},  
      {84, -39.2}, 
      {100, -36.9}, 
      {120, -33.45}, 
      {150, -28.95},
      {180, -28.2},
      {240, -26.1},
      {300, -26.0},
    };

    public static final double[][] kElbowInterpolatorValues = {
      {36, 22},
      {48, 18},
      {50, 14},  
      {84, 10}, 
      {100, 10},
      {120, 10},
    };

    public static final LinearInterpolator kShoulderInterpolator = new LinearInterpolator(kShoulderInterpolatorValues);
    public static final LinearInterpolator kElbowInterpolator = new LinearInterpolator(kElbowInterpolatorValues);

  }

  public static final class ClimberConstants{
    public static final int kClimberServoPWMPort = 0;

    public static final double kClimberServoHooked = 0.0;
    public static final double kClimberServoUnhooked = 0.5;
  }

  public static final class VisionConstants{
    public static final String kCameraName = "ShooterCam";

    public static final Vector<N3> stateStdDevs = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5));
    public static final Vector<N3> VisionSingleTagStdDevs = VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(10));
    public static final Vector<N3> VisionMultiTagStdDevs = VecBuilder.fill(0.25, 0.25, Units.degreesToRadians(5));
    public static final Vector<N3> visionStdDevs = VecBuilder.fill(1, 1, 1);

    public static final double APRILTAG_AMBIGUITY_THRESHOLD = 0.5;
    public static final double POSE_AMBIGUITY_SHIFTER = 0.2;
    public static final double POSE_AMBIGUITY_MULTIPLIER = 1;
    public static final double NOISY_DISTANCE_METERS = 8.25;
    public static final double DISTANCE_WEIGHT = 7;
    public static final int TAG_PRESENCE_WEIGHT = 10;

    public static final AprilTagFieldLayout kAprilTagFieldLayout = AprilTagFields.kDefaultField.loadAprilTagLayoutField();

    public static final Transform3d SHOOTER_CAMERA_TO_ROBOT = new Transform3d(
      new Translation3d(Units.inchesToMeters(10.75)/*camera distance forwards from center*/,
                                                  0.0/*camera distace laterally from center*/,
                        Units.inchesToMeters(2)/*camera distance vertically from ground*/),
      new Rotation3d(Units.degreesToRadians(0)/*leave at 0*/,
                     Units.degreesToRadians(10),/*adjust if camera is mounted at an angle*/
                     Units.degreesToRadians(0)/*shooter is 0, ground intake is 180*/));

    public static final Transform3d PICKUP_CAMERAN_TO_ROBOT = new Transform3d(
      new Translation3d(Units.inchesToMeters(11)/*camera distance forwards from center*/,
                                                  0.0/*camera distace laterally from center*/,
                                                  0.343/*camera distance vertically from ground*/),
                                new Rotation3d(0.0/*leave at 0*/,
                     Units.degreesToRadians(20),/*adjust if camera is mounted at an angle*/
                     Units.degreesToRadians(180)/*shooter is 0, ground intake is 180*/));

    public static final double[][] NoteDistanceInterpolatorValues = {
      {45, 1},
      {10, 2},
      {5, 3},
      {4, 4},
      {2, 5},
      {1.5, 6},
      {1.25, 7},
      {1, 8},
      {0.75, 9},
      {0.6, 10},
      {0.45, 11},
      {0.4, 12}
    };

    public static final double[][] ChaseSpeedInterpolatorValues = {
      {1, 0.2},
      {2, 0.25},
      {3, 0.3},
      {4, 0.35},
      {5, 0.4},
      {6, 0.45},
      {7, 0.5},
      {8, 0.55},
      {9, 0.6},
      {10, 0.65},
      {11, 0.7},
      {12, 0.75}
    };
  }

  public static final class FieldConstants{
    public static final double kFieldLengthMeters = 16.4846;
    public static final double kFieldWidthMeters = 8.1026;
  }

  public static final class LedConstants{
    public static final int kLedStripPWMPort = 9;
    public static final int kLedStripControllerIndex = 0;
    public static final int kLedStripLength = 26;
    public static final int kLedStripOffest = 0;

    public static final BlinkPattern kBlinkPattern = new BlinkPattern();
    public static final ChasePattern kChasePattern = new ChasePattern();
    public static final FadePattern kFadePattern = new FadePattern();
    public static final MergeSortPattern kMergeSortPattern = new MergeSortPattern();
    public static final RainbowPattern kRainbowPattern = new RainbowPattern();
    public static final SolidColorPattern kSolidColorPattern = new SolidColorPattern();
    public static final WavePattern kWavePattern = new WavePattern();

    public static final Color kBrightOrange = new Color(39, 255, 255);
    public static final Color kBrightGreen = new Color(96, 255, 255);
    public static final Color kBrightRed = new Color(0, 255, 255);
    public static final Color kMaroon = new Color(0, 255, 128);
  }
}
