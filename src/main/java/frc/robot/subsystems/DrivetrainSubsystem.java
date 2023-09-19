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
import static frc.robot.Constants.DrivetrainConstants.KINEMATICS;
import static frc.robot.Constants.DrivetrainConstants.PIGEON_ID;
import static frc.robot.Constants.VisionConstants.APRILTAG_CAMERA_NAME;

import java.util.Arrays;
import java.util.Map;
import java.util.concurrent.locks.ReadWriteLock;
import java.util.concurrent.locks.ReentrantReadWriteLock;
import java.util.function.Supplier;
import java.util.stream.IntStream;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
  private static final double UPDATE_FREQUENCY = 250d;
  private static final NetworkTable moduleStatesTable = NetworkTableInstance.getDefault().getTable("SwerveStates");
  private final SwerveDrivePoseEstimator poseEstimator;
  private final Pigeon2 pigeon = new Pigeon2(PIGEON_ID, CANIVORE_BUS_NAME);
  private final SwerveModule[] swerveModules;
  private final Field2d field2d = new Field2d();
  private final Thread photonThread = new Thread(new PhotonRunnable(APRILTAG_CAMERA_NAME, this::addVisionMeasurement));
  private final OdometryThread odometryThread;
  private final ReadWriteLock odometryLock = new ReentrantReadWriteLock();

  private OriginPosition originPosition = kBlueAllianceWallRightSide;
  private boolean sawTag = false;

  private ChassisSpeeds desiredChassisSpeeds;

  private class OdometryThread extends Thread {
    private final BaseStatusSignal[] statusSignals = new BaseStatusSignal[swerveModules.length * 4 + 2];

    public OdometryThread() {
      for (int i = 0; i < swerveModules.length; i++) {
        var moduleSignals = swerveModules[i].getSignals();
        statusSignals[(i * 4) + 0] = moduleSignals[0];
        statusSignals[(i * 4) + 1] = moduleSignals[1];
        statusSignals[(i * 4) + 2] = moduleSignals[2];
        statusSignals[(i * 4) + 3] = moduleSignals[3];
      }
      statusSignals[statusSignals.length - 2] = pigeon.getYaw().clone();
      statusSignals[statusSignals.length - 1] = pigeon.getAccelerationZ().clone();
    }

    @Override
    public void run() {
      for (int i = 0; i < statusSignals.length; i++) {
        statusSignals[i].setUpdateFrequency(UPDATE_FREQUENCY);
      }

      while(true) {
        // Update drivetrain sensor data
        BaseStatusSignal.waitForAll(2.0 / UPDATE_FREQUENCY, statusSignals);
        odometryLock.writeLock().lock();
        try {
          poseEstimator.update(getGyroscopeRotation(), getModulePositions());
        } finally {
          odometryLock.writeLock().unlock();
        }
      }
    }
  }

  public DrivetrainSubsystem() {
    ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
    
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
          KINEMATICS,
          getGyroscopeRotation(),
          getModulePositions(),
          new Pose2d());
      
      odometryThread = new OdometryThread();
      odometryThread.setName("Drivetrain Odometry");
      odometryThread.setDaemon(true);
      odometryThread.start();
        
      // Start PhotonVision thread
      photonThread.setName("PhotonVision");
      photonThread.start();
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
      odometryLock.writeLock().lock();
      try {
        var newPose = flipAlliance(getCurrentPose());
        poseEstimator.resetPosition(getGyroscopeRotation(), getModulePositions(), newPose);
      } finally {
        odometryLock.writeLock().unlock();
      }
    }
  }

  /**
   * Add a vision measurement. Call this with a pose estimate from an AprilTag.
   * @param pose2d Pose estimate based on the vision target (AprilTag)
   * @param timestamp timestamp when the target was seen
   */
  public void addVisionMeasurement(Pose2d pose2d, double timestamp) {
    sawTag = true;
    var visionPose2d = pose2d;
    if (originPosition != kBlueAllianceWallRightSide) {
      visionPose2d = flipAlliance(visionPose2d);
    }
    odometryLock.writeLock().lock();
    try {
      // Update pose estimator
      if (pose2d != null) {
        poseEstimator.addVisionMeasurement(visionPose2d, timestamp);
      }
    } finally {
      odometryLock.writeLock().unlock();
    }
  }

  /**
   * Creates a swerve module instance
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

  private Rotation2d getGyroscopeRotation() {
    return Rotation2d.fromDegrees(
        BaseStatusSignal.getLatencyCompensatedValue(pigeon.getYaw(), pigeon.getAngularVelocityZ()));
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
    return KINEMATICS.toChassisSpeeds(getModuleStates());
  }

  @Override
  public void periodic() {
    // Set the pose on the dashboard
    var dashboardPose = getCurrentPose();
    if (originPosition == kRedAllianceWallRightSide) {
      // Flip the pose when red, since the dashboard field photo cannot be rotated
      dashboardPose = flipAlliance(dashboardPose);
    }
    field2d.setRobotPose(dashboardPose);

    // Set the swerve module states
    if (desiredChassisSpeeds != null) {
      setModuleStates(KINEMATICS.toSwerveModuleStates(desiredChassisSpeeds));
    }

    // Module states for Advantage Scope
    if (DrivetrainConstants.ADD_TO_DASHBOARD) {
      double[] moduleStateArray = new double[swerveModules.length * 2];
      for (int i = 0; i < swerveModules.length; i ++) {
        var module = swerveModules[i];
        moduleStateArray[i * 2] = module.getSteerAngle(false).getRadians();
        moduleStateArray[(i * 2) + 1] = module.getDriveVelocity(false);
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
    odometryLock.readLock().lock();
    try {
      return poseEstimator.getEstimatedPosition();
    } finally {
      odometryLock.readLock().unlock();
    }
  }
  
  /**
   * Resets the current pose to the specified pose. This should ONLY be called
   * when the robot's position on the field is known, like at the beginning of
   * a match.
   * @param newPose new pose
   */
  public void setCurrentPose(Pose2d newPose) {
    odometryLock.writeLock().lock();
    try {
      poseEstimator.resetPosition(getGyroscopeRotation(), getModulePositions(), newPose);
    } finally {
      odometryLock.writeLock().unlock();
    }
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
    return Arrays.stream(swerveModules).map(module -> module.getState(false)).toArray(SwerveModuleState[]::new);
  }

  /**
   * Gets the current drivetrain position, as reported by the modules themselves.
   * @return current drivetrain state. Array orders are frontLeft, frontRight, backLeft, backRight
   */
  private SwerveModulePosition[] getModulePositions() {
    return Arrays.stream(swerveModules).map(module -> module.getPosition(false)).toArray(SwerveModulePosition[]::new);
  }

  /**
   * Sets the states of the modules.
   * @param states array of states. Must be ordered frontLeft, frontRight, backLeft, backRight
   */
  public void setModuleStates(SwerveModuleState[] states) {
    double[] moduleSetpointArray = new double[states.length * 2];
    SwerveDriveKinematics.desaturateWheelSpeeds(states, DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND);
    IntStream.range(0, swerveModules.length).forEach(i -> {
      var swerveModule = swerveModules[i];
      var desiredState = SwerveModuleState.optimize(states[i], swerveModule.getSteerAngle(false));
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
            KINEMATICS,
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
      KINEMATICS,
      new PIDController(TRANSLATION_kP, TRANSLATION_kI, TRANSLATION_kD),
      new PIDController(TRANSLATION_kP, TRANSLATION_kI, TRANSLATION_kD),
      thetaController,
      this::setModuleStates,
      useAllianceColor,
      this
    );

    return ppSwerveCommand;
  }

  private void setBrakeMode(boolean brakeMode) {
    for (SwerveModule swerveModule : swerveModules) {
      swerveModule.setBrakeMode(brakeMode);
    }
  }

}
