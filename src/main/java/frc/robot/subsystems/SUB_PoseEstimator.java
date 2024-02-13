package frc.robot.subsystems;

import static edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide;
import static edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition.kRedAllianceWallRightSide;

import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;

import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CameraConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;

public class SUB_PoseEstimator extends SubsystemBase {

  // Kalman Filter Configuration. These can be "tuned-to-taste" based on how much
  // you trust your various sensors. Smaller numbers will cause the filter to
  // "trust" the estimate from that particular component more than the others.
  // This in turn means the particualr component will have a stronger influence
  // on the final pose estimate.
  private final Supplier<Rotation2d> rotationSupplier;
  private final Supplier<SwerveModulePosition[]> modulePositionSupplier;
  private final SwerveDrivePoseEstimator poseEstimator;
  private final Field2d field2d = new Field2d();
  private final PhotonRunnable ShooterEstimator = new PhotonRunnable(new PhotonCamera("shooter_camera"),
      CameraConstants.SHOOTER_CAMERA_TO_ROBOT);
  private final PhotonRunnable Pickup = new PhotonRunnable(new PhotonCamera("pickup_camera"),
      CameraConstants.PICKUP_CAMERAN_TO_ROBOT);
  private SUB_Drivetrain m_drivetrain;
  // private final PhotonRunnable backEstimator = new PhotonRunnable(new
  // PhotonCamera("backCamera"),
  // VisionROBOT_TO_BACK_CAMERA);

    /**
   * Standard deviations of model states. Increase these numbers to trust your model's state estimates less. This
   * matrix is in the form [x, y, theta]ᵀ, with units in meters and radians, then meters.
   */
  private static final Vector<N3> stateStdDevs = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5));
  
  /**
   * Standard deviations of the vision measurements. Increase these numbers to trust global measurements from vision
   * less. This matrix is in the form [x, y, theta]ᵀ, with units in meters and radians.
   */
  private static final Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(10));

  // private final Notifier rightNotifier = new Notifier(ShooterEstimator);
  // private final Notifier leftNotifier = new Notifier(Pickup);
  private final Notifier allNotifier = new Notifier(() -> {
    ShooterEstimator.run();
    Pickup.run();
  });
  // private final Notifier backNotifier = new Notifier(backEstimator);

  private OriginPosition originPosition = kBlueAllianceWallRightSide;

  // private final ArrayList<Double> xValues = new ArrayList<Double>();
  // private final ArrayList<Double> yValues = new ArrayList<Double>();

  public SUB_PoseEstimator(SUB_Drivetrain p_Drivetrain, Supplier<Rotation2d> rotationSupplier,
         Supplier<SwerveModulePosition[]> modulePositionSupplier) {
    m_drivetrain = p_Drivetrain;
    
    this.rotationSupplier = rotationSupplier;
    this.modulePositionSupplier = modulePositionSupplier;


    this.poseEstimator = new SwerveDrivePoseEstimator(
        DriveConstants.kDriveKinematics,
        rotationSupplier.get(),
        modulePositionSupplier.get(),
        new Pose2d(),
        stateStdDevs,
        visionMeasurementStdDevs);

    // Start PhotonVision thread
    // rightNotifier.setName("rightRunnable");
    // rightNotifier.startPeriodic(0.02);

    // // Start PhotonVision thread
    // leftNotifier.setName("leftRunnable");
    // leftNotifier.startPeriodic(0.02);

    allNotifier.setName("runAll");
    allNotifier.startPeriodic(0.02);

    // backNotifier.setName("backRunnable");
    // backNotifier.startPeriodic(0.02);
    ShuffleboardTab tab = Shuffleboard.getTab("SmartDashboard");
    addDashboardWidgets(tab);
  }

  public void addDashboardWidgets(ShuffleboardTab tab) {
    tab.add("Field", field2d).withPosition(0, 0).withSize(6, 4);
    tab.addString("Pose", this::getFomattedPose).withPosition(6, 2).withSize(2, 1);
  }

  /**
   * Sets the alliance. This is used to configure the origin of the AprilTag map
   * 
   * @param alliance alliance
   */
  public void setAlliance(Alliance alliance) {
    boolean allianceChanged = false;
    switch (alliance) {
      case Blue:
        allianceChanged = (originPosition == kRedAllianceWallRightSide);
        originPosition = kBlueAllianceWallRightSide;
        break;
      case Red:
        allianceChanged = (originPosition == kBlueAllianceWallRightSide);
        originPosition = kRedAllianceWallRightSide;
        break;
      default:
        // No valid alliance data. Nothing we can do about it
    }

    if (allianceChanged) {
      // The alliance changed, which changes the coordinate system.
      // Since a tag was seen, and the tags are all relative to the coordinate system,
      // the estimated pose
      // needs to be transformed to the new coordinate system.
      var newPose = flipAlliance(getCurrentPose());
      poseEstimator.resetPosition(rotationSupplier.get(), modulePositionSupplier.get(), newPose);
    }
  }

  @Override
  public void periodic() {
    // Update pose estimator with drivetrain sensors
    poseEstimator.update(rotationSupplier.get(), modulePositionSupplier.get());
    // if (true) {
      estimatorChecker(ShooterEstimator);
      estimatorChecker(Pickup);
    // } else {
    //   allNotifier.close();
    // }

    // estimatorChecker(backEstimator);

    // Set the pose on the dashboard
    var dashboardPose = poseEstimator.getEstimatedPosition();
    if (originPosition == kRedAllianceWallRightSide) {
      // Flip the pose when red, since the dashboard field photo cannot be rotated
      dashboardPose = flipAlliance(dashboardPose);
    }
    field2d.setRobotPose(dashboardPose);

    SmartDashboard.putNumber("vision estimated x", getCurrentPose().getX());
    SmartDashboard.putNumber("vision estimated Y", getCurrentPose().getY());
    SmartDashboard.putNumber("vision estimated heading", getCurrentPose().getRotation().getDegrees());
    m_drivetrain.resetPose(getCurrentPose());
  }

  private String getFomattedPose() {
    var pose = getCurrentPose();
    return String.format("(%.3f, %.3f) %.2f degrees",
        pose.getX(),
        pose.getY(),
        pose.getRotation().getDegrees());
  }

  public Pose2d getCurrentPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /**
   * Resets the current pose to the specified pose. This should ONLY be called
   * when the robot's position on the field is known, like at the beginning of
   * a match.
   * 
   * @param newPose new pose
   */
  public void setCurrentPose(Pose2d newPose) {
    poseEstimator.resetPosition(rotationSupplier.get(), modulePositionSupplier.get(), newPose);
  }

  /**
   * Resets the position on the field to 0,0 0-degrees, with forward being
   * downfield. This resets
   * what "forward" is for field oriented driving.
   */
  public void resetFieldPosition() {
    setCurrentPose(new Pose2d());
  }

  /**
   * Transforms a pose to the opposite alliance's coordinate system. (0,0) is
   * always on the right corner of your
   * alliance wall
   * @param poseToFlip pose to transform to the other alliance
   * @return pose relative to the other alliance's coordinate system
   */
  private Pose2d flipAlliance(Pose2d poseToFlip) {
    return poseToFlip.relativeTo(new Pose2d(
        new Translation2d(FieldConstants.kFieldLengthMeters, FieldConstants.kFieldWidthMeters),
        new Rotation2d(Math.PI)));
  }

  private Matrix<N3, N1> confidenceCalculator(EstimatedRobotPose estimation) {
    double smallestDistance = Double.POSITIVE_INFINITY;
    for (var target : estimation.targetsUsed) {
      var t3d = target.getBestCameraToTarget();
      var distance = Math.sqrt(Math.pow(t3d.getX(), 2) + Math.pow(t3d.getY(), 2) + Math.pow(t3d.getZ(), 2));
      if (distance < smallestDistance)
        smallestDistance = distance;
    }
    double poseAmbiguityFactor = estimation.targetsUsed.size() != 1
        ? 1
        : Math.max(
            1,
            (estimation.targetsUsed.get(0).getPoseAmbiguity()
                + CameraConstants.POSE_AMBIGUITY_SHIFTER)
                * CameraConstants.POSE_AMBIGUITY_MULTIPLIER);
    double confidenceMultiplier = Math.max(
        1,
        (Math.max(
            1,
            Math.max(0, smallestDistance - CameraConstants.NOISY_DISTANCE_METERS)
                * CameraConstants.DISTANCE_WEIGHT)
            * poseAmbiguityFactor)
            / (1
                + ((estimation.targetsUsed.size() - 1) * CameraConstants.TAG_PRESENCE_WEIGHT)));

    return visionMeasurementStdDevs.times(confidenceMultiplier);
  }

  public void estimatorChecker(PhotonRunnable estimator) {
    var cameraPose = estimator.grabLatestEstimatedPose();
    if (cameraPose != null) {
      // New pose from vision
      var pose2d = cameraPose.estimatedPose.toPose2d();
      if (originPosition == kRedAllianceWallRightSide) {
        pose2d = flipAlliance(pose2d);
      }
      poseEstimator.addVisionMeasurement(pose2d, cameraPose.timestampSeconds, confidenceCalculator(cameraPose));
    }
  }
}