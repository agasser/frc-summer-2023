// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide;
import static edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition.kRedAllianceWallRightSide;
import static frc.robot.Constants.AutoDriveConstants.THETA_CONSTRAINTS;
import static frc.robot.Constants.AutoDriveConstants.TRANSLATION_kD;
import static frc.robot.Constants.AutoDriveConstants.TRANSLATION_kI;
import static frc.robot.Constants.AutoDriveConstants.TRANSLATION_kP;
import static frc.robot.Constants.DrivetrainConstants.BACK_LEFT_MODULE_DRIVE_MOTOR;
import static frc.robot.Constants.DrivetrainConstants.BACK_LEFT_MODULE_STEER_ENCODER;
import static frc.robot.Constants.DrivetrainConstants.BACK_LEFT_MODULE_STEER_MOTOR;
import static frc.robot.Constants.DrivetrainConstants.BACK_LEFT_MODULE_STEER_OFFSET;
import static frc.robot.Constants.DrivetrainConstants.BACK_RIGHT_MODULE_DRIVE_MOTOR;
import static frc.robot.Constants.DrivetrainConstants.BACK_RIGHT_MODULE_STEER_ENCODER;
import static frc.robot.Constants.DrivetrainConstants.BACK_RIGHT_MODULE_STEER_MOTOR;
import static frc.robot.Constants.DrivetrainConstants.BACK_RIGHT_MODULE_STEER_OFFSET;
import static frc.robot.Constants.DrivetrainConstants.CANIVORE_BUS_NAME;
import static frc.robot.Constants.DrivetrainConstants.FRONT_LEFT_MODULE_DRIVE_MOTOR;
import static frc.robot.Constants.DrivetrainConstants.FRONT_LEFT_MODULE_STEER_ENCODER;
import static frc.robot.Constants.DrivetrainConstants.FRONT_LEFT_MODULE_STEER_MOTOR;
import static frc.robot.Constants.DrivetrainConstants.FRONT_LEFT_MODULE_STEER_OFFSET;
import static frc.robot.Constants.DrivetrainConstants.FRONT_RIGHT_MODULE_DRIVE_MOTOR;
import static frc.robot.Constants.DrivetrainConstants.FRONT_RIGHT_MODULE_STEER_ENCODER;
import static frc.robot.Constants.DrivetrainConstants.FRONT_RIGHT_MODULE_STEER_MOTOR;
import static frc.robot.Constants.DrivetrainConstants.FRONT_RIGHT_MODULE_STEER_OFFSET;
import static frc.robot.Constants.DrivetrainConstants.PIGEON_ID;
import static frc.robot.Constants.VisionConstants.APRILTAG_CAMERA_NAME;

import java.util.Arrays;
import java.util.Map;
import java.util.function.Supplier;
import java.util.stream.IntStream;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutoDriveConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.PhotonRunnable;
import frc.robot.swerve.ModuleConfiguration;
import frc.robot.swerve.SwerveModule;
import frc.robot.swerve.SwerveSpeedController;
import frc.robot.swerve.SwerveSteerController;

public class DrivetrainSubsystem extends SubsystemBase {

  private final Pigeon2 pigeon = new Pigeon2(PIGEON_ID, CANIVORE_BUS_NAME);
  private final SwerveModule[] swerveModules;

  /**
   * Standard deviations of model states. Increase these numbers to trust your model's state estimates less. This
   * matrix is in the form [x, y, theta]ᵀ, with units in meters and radians, then meters.
   */
  private static final Vector<N3> stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.1);

  /**
   * Standard deviations of the vision measurements. Increase these numbers to trust global measurements from vision
   * less. This matrix is in the form [x, y, theta]ᵀ, with units in meters and radians.
   */
  private static final Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(1.5, 1.5, 1.5);

  private final SwerveDrivePoseEstimator poseEstimator;

  private final Field2d field2d = new Field2d();
  private final PhotonRunnable photonEstimator = new PhotonRunnable(APRILTAG_CAMERA_NAME);
  private final Notifier photonNotifier = new Notifier(photonEstimator);

  private OriginPosition originPosition = kBlueAllianceWallRightSide;
  private boolean sawTag = false;

  private static final NetworkTable moduleStatesTable = NetworkTableInstance.getDefault().getTable("SwerveStates");

  private ChassisSpeeds desiredChassisSpeeds;

  public DrivetrainSubsystem() {
    ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
    
    var pigeonConfig = new Pigeon2Configuration();
    pigeonConfig.MountPose.MountPoseYaw = 0;
    pigeonConfig.MountPose.MountPosePitch = 0;
    pigeonConfig.MountPose.MountPoseRoll = 0;
    pigeon.getConfigurator().apply(pigeonConfig);

    ShuffleboardLayout frontLeftLayout = null;
    ShuffleboardLayout frontRightLayout = null;
    ShuffleboardLayout backLeftLayout = null;
    ShuffleboardLayout backRightLayout = null;

    if (DrivetrainConstants.ADD_TO_DASHBOARD) {
      frontLeftLayout = tab.getLayout("Front Left Module", BuiltInLayouts.kGrid)
          .withProperties(Map.of("Number of columns", 1, "Number of rows", 6))
          .withSize(2, 4)
          .withPosition(0, 0);

      frontRightLayout = tab.getLayout("Front Right Module", BuiltInLayouts.kGrid)
          .withProperties(Map.of("Number of columns", 1, "Number of rows", 6))
          .withSize(2, 4)
          .withPosition(2, 0);
      
      backLeftLayout = tab.getLayout("Back Left Module", BuiltInLayouts.kGrid)
          .withProperties(Map.of("Number of columns", 1, "Number of rows", 6))
          .withSize(2, 4)
          .withPosition(4, 0);
      
      backRightLayout = tab.getLayout("Back Right Module", BuiltInLayouts.kGrid)
          .withProperties(Map.of("Number of columns", 1, "Number of rows", 6))
          .withSize(2, 4)
          .withPosition(6, 0);
    }

    swerveModules = new SwerveModule[] {
        createSwerveModule(
            frontLeftLayout,
            ModuleConfiguration.MK4I_L2,
            FRONT_LEFT_MODULE_DRIVE_MOTOR,
            FRONT_LEFT_MODULE_STEER_MOTOR,
            FRONT_LEFT_MODULE_STEER_ENCODER,
            FRONT_LEFT_MODULE_STEER_OFFSET
        ),
        createSwerveModule(
            frontRightLayout,
            ModuleConfiguration.MK4I_L2,
            FRONT_RIGHT_MODULE_DRIVE_MOTOR,
            FRONT_RIGHT_MODULE_STEER_MOTOR,
            FRONT_RIGHT_MODULE_STEER_ENCODER,
            FRONT_RIGHT_MODULE_STEER_OFFSET
        ),
        createSwerveModule(
            backLeftLayout,
            ModuleConfiguration.MK4I_L2,
            BACK_LEFT_MODULE_DRIVE_MOTOR,
            BACK_LEFT_MODULE_STEER_MOTOR,
            BACK_LEFT_MODULE_STEER_ENCODER,
            BACK_LEFT_MODULE_STEER_OFFSET
        ),
        createSwerveModule(
            backRightLayout,
            ModuleConfiguration.MK4I_L2,
            BACK_RIGHT_MODULE_DRIVE_MOTOR,
            BACK_RIGHT_MODULE_STEER_MOTOR,
            BACK_RIGHT_MODULE_STEER_ENCODER,
            BACK_RIGHT_MODULE_STEER_OFFSET
        )};

      new Trigger(RobotState::isEnabled)
           .onTrue(runOnce(() -> setBrakeMode(true)))
           .onFalse(runOnce(() -> setBrakeMode(false)).ignoringDisable(true));
  
      poseEstimator =  new SwerveDrivePoseEstimator(
          DrivetrainConstants.KINEMATICS,
          getGyroscopeRotation(),
          getModulePositions(),
          new Pose2d(),
          stateStdDevs,
          visionMeasurementStdDevs);
        
      // Start PhotonVision thread
      photonNotifier.setName("PhotonRunnable");
      photonNotifier.startPeriodic(.02);
  }

  public void addDashboardWidgets(ShuffleboardTab tab) {
    tab.add("Field", field2d).withPosition(0, 0).withSize(6, 4);
    tab.addString("Pose", this::getFomattedPose).withPosition(6, 2).withSize(2, 1);
  }

  /**
   * Sets the alliance. This is used to configure the origin of the AprilTag map
   * @param alliance alliance
   */
  public void setAlliance(Alliance alliance) {
    boolean allianceChanged = false;
    switch(alliance) {
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

    if (allianceChanged && sawTag) {
      // The alliance changed, which changes the coordinate system.
      // Since a tag was seen, and the tags are all relative to the coordinate system, the estimated pose
      // needs to be transformed to the new coordinate system.
      var newPose = flipAlliance(getCurrentPose());
      poseEstimator.resetPosition(getGyroscopeRotation(), getModulePositions(), newPose);
    }
  }

  /**
   * Creates a server module instance
   * @param container shuffleboard layout, or null
   * @param moduleConfiguration module configuration
   * @param driveMotorPort drive motor CAN ID
   * @param steerMotorPort steer motor CAN ID
   * @param steerEncoderPort steer encoder CAN ID
   * @param steerOffset offset for steer encoder
   * @return new swerve module instance
   */
  private static SwerveModule createSwerveModule(
      ShuffleboardLayout container,
      ModuleConfiguration moduleConfiguration,
      int driveMotorPort,
      int steerMotorPort,
      int steerEncoderPort,
      double steerOffset) {

    return new SwerveModule(
        new SwerveSpeedController(driveMotorPort, moduleConfiguration, container), 
        new SwerveSteerController(steerMotorPort, steerEncoderPort, steerOffset, container, moduleConfiguration));
  }

  public Rotation2d getGyroscopeRotation() {
    return pigeon.getRotation2d();
  }

  /**
   * Sets the desired chassis speeds
   * @param chassisSpeeds desired chassis speeds
   */
  public void drive(ChassisSpeeds chassisSpeeds) {
    desiredChassisSpeeds = chassisSpeeds;
  }

  /**
   * Set the wheels to an X pattern to plant the robot.
   */
  public void setWheelsToX() {
    desiredChassisSpeeds = null;
    setModuleStates(new SwerveModuleState[] {
      // front left
      new SwerveModuleState(0.0, Rotation2d.fromDegrees(45.0)),
      // front right
      new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45.0)),
      // back left
      new SwerveModuleState(0.0, Rotation2d.fromDegrees(135.0)),
      // back right
      new SwerveModuleState(0.0, Rotation2d.fromDegrees(-135.0))
    });
  }

  /**
   * Sets the desired speeds to zero
   */
  public void stop() {
    drive(new ChassisSpeeds());
  }

  /**
   * Gets the actual chassis speeds
   * @return actual chassis speeds
   */
  public ChassisSpeeds getChassisSpeeds() {
    return DrivetrainConstants.KINEMATICS.toChassisSpeeds(getModuleStates());
  }

  @Override
  public void periodic() {
    
    // Update pose estimator with drivetrain sensors
    poseEstimator.update(getGyroscopeRotation(), getModulePositions());

    var visionPose = photonEstimator.grabLatestEstimatedPose();
    if (visionPose != null) {
      // New pose from vision
      sawTag = true;
      var pose2d = visionPose.estimatedPose.toPose2d();
      if (originPosition != kBlueAllianceWallRightSide) {
        pose2d = flipAlliance(pose2d);
      }
      poseEstimator.addVisionMeasurement(pose2d, visionPose.timestampSeconds);
    }

    // Set the pose on the dashboard
    var dashboardPose = poseEstimator.getEstimatedPosition();
    if (originPosition == kRedAllianceWallRightSide) {
      // Flip the pose when red, since the dashboard field photo cannot be rotated
      dashboardPose = flipAlliance(dashboardPose);
    }
    field2d.setRobotPose(dashboardPose);

    // Set the swerve module states
    if (desiredChassisSpeeds != null) {
      var desiredStates = DrivetrainConstants.KINEMATICS.toSwerveModuleStates(desiredChassisSpeeds);
      SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND);
      setModuleStates(desiredStates);
    }
    // Module states for Advantage Scope
    if (DrivetrainConstants.ADD_TO_DASHBOARD) {
      double[] moduleStateArray = new double[swerveModules.length * 2];
      for (int i = 0; i < swerveModules.length; i ++) {
        var module = swerveModules[i];
        moduleStateArray[i * 2] = module.getSteerAngle().getRadians();
        moduleStateArray[(i * 2) + 1] = module.getDriveVelocity();
      }
      moduleStatesTable.getEntry("Measured").setDoubleArray(moduleStateArray);
      moduleStatesTable.getEntry("Rotation").setDouble(dashboardPose.getRotation().getRadians());
    }
    // Always reset desiredChassisSpeeds to null to prevent latching to the last state (aka motor safety)!!
    desiredChassisSpeeds = null;

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
   * @param newPose new pose
   */
  public void setCurrentPose(Pose2d newPose) {
    poseEstimator.resetPosition(getGyroscopeRotation(), getModulePositions(), newPose);
  }

  /**
   * Resets the position on the field to 0,0 0-degrees, with forward being downfield. This resets
   * what "forward" is for field oriented driving.
   */
  public void resetFieldPosition() {
    setCurrentPose(new Pose2d());
  }

  /**
   * Transforms a pose to the opposite alliance's coordinate system. (0,0) is always on the right corner of your
   * alliance wall, so for 2023, the field elements are at different coordinates for each alliance.
   * @param poseToFlip pose to transform to the other alliance
   * @return pose relative to the other alliance's coordinate system
   */
  private Pose2d flipAlliance(Pose2d poseToFlip) {
    return poseToFlip.relativeTo(VisionConstants.FLIPPING_POSE);
  }

  /**
   * Gets the current drivetrain state (velocity, and angle), as reported by the modules themselves.
   * @return current drivetrain state. Array orders are frontLeft, frontRight, backLeft, backRight
   */
  private SwerveModuleState[] getModuleStates() {
    return Arrays.stream(swerveModules).map(module -> module.getState()).toArray(SwerveModuleState[]::new);
  }

  /**
   * Gets the current drivetrain position, as reported by the modules themselves.
   * @return current drivetrain state. Array orders are frontLeft, frontRight, backLeft, backRight
   */
  public SwerveModulePosition[] getModulePositions() {
    return Arrays.stream(swerveModules).map(module -> module.getPosition()).toArray(SwerveModulePosition[]::new);
  }

  /**
   * Sets the states of the modules.
   * @param states array of states. Must be ordered frontLeft, frontRight, backLeft, backRight
   */
  public void setModuleStates(SwerveModuleState[] states) {
    double[] moduleSetpointArray = new double[states.length * 2];
    IntStream.range(0, swerveModules.length).forEach(i -> {
      var swerveModule = swerveModules[i];
      var desiredState = SwerveModuleState.optimize(states[i], swerveModule.getSteerAngle());
      swerveModule.setDesiredState(desiredState);
      
      // Module setpoints for Advantage Scope
      if (DrivetrainConstants.ADD_TO_DASHBOARD) {
        moduleSetpointArray[i * 2] = desiredState.angle.getRadians();
        moduleSetpointArray[(i * 2) + 1] = desiredState.speedMetersPerSecond;
      }
    });
    if (DrivetrainConstants.ADD_TO_DASHBOARD) {
      moduleStatesTable.getEntry("Setpoints").setDoubleArray(moduleSetpointArray);
    }
  }

  /**
   * Creates a command to follow a Trajectory on the drivetrain.
   * @param trajectory trajectory to follow
   * @return command that will run the trajectory
   */
  public Command createCommandForTrajectory(Trajectory trajectory, Supplier<Pose2d> poseSupplier) {
    var thetaController = new ProfiledPIDController(
        AutoDriveConstants.THETA_kP, AutoDriveConstants.THETA_kI, AutoDriveConstants.THETA_kD, THETA_CONSTRAINTS);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand =
        new SwerveControllerCommand(
            trajectory,
            poseSupplier,
            DrivetrainConstants.KINEMATICS,
            new PIDController(TRANSLATION_kP, TRANSLATION_kI, TRANSLATION_kD),
            new PIDController(TRANSLATION_kP, TRANSLATION_kI, TRANSLATION_kD),
            thetaController,
            this::setModuleStates,
            this);
            
      return swerveControllerCommand;
  }

  public Command createCommandForTrajectory(
        PathPlannerTrajectory trajectory, Supplier<Pose2d> poseSupplier, boolean useAllianceColor) {
          
    var thetaController = new PIDController(AutoDriveConstants.THETA_kP, AutoDriveConstants.THETA_kI, AutoDriveConstants.THETA_kD);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    
    var ppSwerveCommand = new PPSwerveControllerCommand(
      trajectory, 
      poseSupplier,
      DrivetrainConstants.KINEMATICS,
      new PIDController(TRANSLATION_kP, TRANSLATION_kI, TRANSLATION_kD),
      new PIDController(TRANSLATION_kP, TRANSLATION_kI, TRANSLATION_kD),
      thetaController,
      this::setModuleStates,
      useAllianceColor,
      this
    );

    return ppSwerveCommand;
  }

  public void setBrakeMode(boolean brakeMode) {
    for (SwerveModule swerveModule : swerveModules) {
      swerveModule.setBrakeMode(brakeMode);
    }
  }

}
