// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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

import java.util.Arrays;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoDriveConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Robot;
import frc.robot.swerve.ModuleConfiguration;
import frc.robot.swerve.SwerveModule;
import frc.robot.swerve.SwerveModuleIO;
import frc.robot.swerve.SwerveModuleIOInputsAutoLogged;
import frc.robot.swerve.SwerveSpeedController;
import frc.robot.swerve.SwerveSteerController;

public class DrivetrainSubsystem extends SubsystemBase {

  private final WPI_Pigeon2 pigeon = new WPI_Pigeon2(PIGEON_ID, CANIVORE_BUS_NAME);
  private final SwerveModuleIO[] swerveModules;

  private final SwerveModuleIOInputsAutoLogged[] swerveModuleIOInputs = {
    new SwerveModuleIOInputsAutoLogged(), new SwerveModuleIOInputsAutoLogged(),
    new SwerveModuleIOInputsAutoLogged(), new SwerveModuleIOInputsAutoLogged()
  };

  private ChassisSpeeds desiredChassisSpeeds;

  public DrivetrainSubsystem() {
    pigeon.configMountPoseRoll(0);
    pigeon.configMountPoseYaw(0);
    pigeon.configMountPosePitch(0);

    if (Robot.isReal()) {
      swerveModules = new SwerveModule[] {
          createSwerveModule(
              ModuleConfiguration.MK4I_L2,
              FRONT_LEFT_MODULE_DRIVE_MOTOR,
              FRONT_LEFT_MODULE_STEER_MOTOR,
              FRONT_LEFT_MODULE_STEER_ENCODER,
              FRONT_LEFT_MODULE_STEER_OFFSET,
              DrivetrainConstants.DRIVE_kS,
              DrivetrainConstants.DRIVE_kV,
              DrivetrainConstants.DRIVE_kA
          ),
          createSwerveModule(
              ModuleConfiguration.MK4I_L2,
              FRONT_RIGHT_MODULE_DRIVE_MOTOR,
              FRONT_RIGHT_MODULE_STEER_MOTOR,
              FRONT_RIGHT_MODULE_STEER_ENCODER,
              FRONT_RIGHT_MODULE_STEER_OFFSET,
              DrivetrainConstants.DRIVE_kS,
              DrivetrainConstants.DRIVE_kV + 0.075,
              DrivetrainConstants.DRIVE_kA
          ),
          createSwerveModule(
              ModuleConfiguration.MK4I_L2,
              BACK_LEFT_MODULE_DRIVE_MOTOR,
              BACK_LEFT_MODULE_STEER_MOTOR,
              BACK_LEFT_MODULE_STEER_ENCODER,
              BACK_LEFT_MODULE_STEER_OFFSET,
              DrivetrainConstants.DRIVE_kS,
              DrivetrainConstants.DRIVE_kV,
              DrivetrainConstants.DRIVE_kA
          ),
          createSwerveModule(
              ModuleConfiguration.MK4I_L2,
              BACK_RIGHT_MODULE_DRIVE_MOTOR,
              BACK_RIGHT_MODULE_STEER_MOTOR,
              BACK_RIGHT_MODULE_STEER_ENCODER,
              BACK_RIGHT_MODULE_STEER_OFFSET,
              DrivetrainConstants.DRIVE_kS,
              DrivetrainConstants.DRIVE_kV + 0.075,
              DrivetrainConstants.DRIVE_kA
          )};
      } else {
        swerveModules = new SwerveModuleIO[] {
          new SwerveModuleIO(){}, new SwerveModuleIO(){}, new SwerveModuleIO(){}, new SwerveModuleIO(){}
        };
      }

      // Put all the modules into brake mode
      for (SwerveModuleIO swerveModule : swerveModules) {
        swerveModule.setNeutralMode(NeutralMode.Brake);
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
      ModuleConfiguration moduleConfiguration,
      int driveMotorPort,
      int steerMotorPort,
      int steerEncoderPort,
      double steerOffset,
      double kS,
      double kV,
      double kA) {

    return new SwerveModule(
        new SwerveSpeedController(driveMotorPort, moduleConfiguration, kS, kV, kA), 
        new SwerveSteerController(steerMotorPort, steerEncoderPort, steerOffset, moduleConfiguration));
  }

  public Rotation2d getGyroscopeRotation() {
    return pigeon.getRotation2d();
  }

  /**
   * Gets the raw gyro data.
   * @return x[0], y[1], and z[2] data in degrees per second
   */
  public double[] getGyroVelocityXYZ() {
    double[] xyz = new double[3];
    pigeon.getRawGyro(xyz);
    return xyz;
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
    for(int i = 0; i < swerveModules.length; i ++) {
      swerveModules[i].updateInputs(swerveModuleIOInputs[i]);
      Logger.getInstance().processInputs("Swerve_" + i, swerveModuleIOInputs[i]);
    }

    // Set the swerve module states
    if (desiredChassisSpeeds != null) {
      var desiredStates = DrivetrainConstants.KINEMATICS.toSwerveModuleStates(desiredChassisSpeeds);
      SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND);
      setModuleStates(desiredStates);
    }
    Logger.getInstance().recordOutput("MeasuredSwerveStates", getModuleStates());

    // Always reset desiredChassisSpeeds to null to prevent latching to the last state (aka motor safety)!!
    desiredChassisSpeeds = null;
  }

  /**
   * Gets the current drivetrain state (velocity, and angle), as reported by the modules themselves.
   * @return current drivetrain state. Array orders are frontLeft, frontRight, backLeft, backRight
   */
  private SwerveModuleState[] getModuleStates() {
    return Arrays.stream(swerveModuleIOInputs)
        .map(input -> new SwerveModuleState(input.driveVelocityMetersPerSecond, new Rotation2d(input.steerAngleRadians)))
        .toArray(SwerveModuleState[]::new);
  }

  /**
   * Gets the current drivetrain position, as reported by the modules themselves.
   * @return current drivetrain state. Array orders are frontLeft, frontRight, backLeft, backRight
   */
  public SwerveModulePosition[] getModulePositions() {
    return Arrays.stream(swerveModuleIOInputs)
        .map(input -> new SwerveModulePosition(input.drivePositionMeters, new Rotation2d(input.steerAngleRadians)))
        .toArray(SwerveModulePosition[]::new);
  }

  /**
   * Sets the states of the modules.
   * @param states array of states. Must be ordered frontLeft, frontRight, backLeft, backRight
   */
  public void setModuleStates(SwerveModuleState[] states) {
    var optimizedStates = new SwerveModuleState[4];
    for(int i = 0; i < swerveModules.length; i++) {
      optimizedStates[i] =
          SwerveModuleState.optimize(states[i], new Rotation2d(swerveModuleIOInputs[i].steerAngleRadians));
      swerveModules[i].setDesiredState(optimizedStates[i]);
    }
    Logger.getInstance().recordOutput("SwerveStateSetpoints", optimizedStates);
  }

  /**
   * Reseeds the Talon FX steer motors from their CANCoder absolute position. Workaround for "dead wheel"
   */
  public void reseedSteerMotorOffsets() {
    Arrays.stream(swerveModules).forEach(SwerveModuleIO::reseedSteerMotorOffset);
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

}
