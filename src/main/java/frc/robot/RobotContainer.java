// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.run;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.robot.commands.FieldOrientedDriveCommand;
import frc.robot.commands.PoseEstimatorCommand;
import frc.robot.controls.ControlBindings;
import frc.robot.controls.JoystickControlBindings;
import frc.robot.controls.XBoxControlBindings;
import frc.robot.subsystems.DrivetrainSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private final ControlBindings controlBindings;

  private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem();
  private final PoseEstimatorCommand poseEstimator =
      new PoseEstimatorCommand(drivetrainSubsystem::getGyroscopeRotation, drivetrainSubsystem::getModulePositions);
  
  private final FieldOrientedDriveCommand fieldOrientedDriveCommand;

  private final Timer reseedTimer = new Timer();

  private final AutonomousBuilder autoBuilder =
      new AutonomousBuilder(drivetrainSubsystem, poseEstimator::getCurrentPose, poseEstimator::setCurrentPose);
  
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure control binding scheme
    if (DriverStation.getJoystickIsXbox(0)) {
      controlBindings = new XBoxControlBindings();
    } else {
      controlBindings = new JoystickControlBindings();
    }

    // Schedule the pose estimator (repeatedly in case it unexpectedly stops)
    poseEstimator.repeatedly().schedule();

    fieldOrientedDriveCommand = new FieldOrientedDriveCommand(
        drivetrainSubsystem,
        () -> poseEstimator.getCurrentPose().getRotation(),
        controlBindings.translationX(),
        controlBindings.translationY(),
        controlBindings.omega());

    configureDefaultCommands();
    configureButtonBindings();
    configureDashboard();
    reseedTimer.start();
  }

  private void configureDefaultCommands() {
    // Set up the default command for the drivetrain.
    drivetrainSubsystem.setDefaultCommand(fieldOrientedDriveCommand);
  }

  private void configureDashboard() {
    /**** Driver tab ****/
    var driverTab = Shuffleboard.getTab("Driver");
    autoBuilder.addDashboardWidgets(driverTab);
    
    /**** Vision tab ****/
    final var visionTab = Shuffleboard.getTab("Vision");

    // Pose estimation
    poseEstimator.addDashboardWidgets(visionTab);
  }

  private void configureButtonBindings() {
    // reset the robot pose
    controlBindings.resetPose().ifPresent(trigger -> trigger.onTrue(runOnce(poseEstimator::resetFieldPosition)));
    // Start button reseeds the steer motors to fix dead wheel
    controlBindings.reseedSteerMotors()
        .ifPresent(trigger -> trigger.onTrue(drivetrainSubsystem.runOnce(drivetrainSubsystem::reseedSteerMotorOffsets)
            .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)));

    // POV Right to put the wheels in an X until the driver tries to drive
    controlBindings.wheelsToX()
        .ifPresent(trigger -> trigger.onTrue(run(drivetrainSubsystem::setWheelsToX, drivetrainSubsystem)
            .until(controlBindings.driverWantsControl())));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoBuilder.getAutonomousCommand();
  }

  public void disabledPeriodic() {
    // Reseed the motor offset continuously when the robot is disabled to help solve dead wheel issue
    // if (reseedTimer.advanceIfElapsed(1.0)) {
    //   drivetrainSubsystem.reseedSteerMotorOffsets();
    // }
  }

  /**
   * Called when the alliance reported by the driverstation/FMS changes.
   * @param alliance new alliance value
   */
  public void onAllianceChanged(Alliance alliance) {
    poseEstimator.setAlliance(alliance);
  }

}
