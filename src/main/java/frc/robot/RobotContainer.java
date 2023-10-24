// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.run;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoDriveConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.commands.FieldOrientedDriveCommand;
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
  
  private final FieldOrientedDriveCommand fieldOrientedDriveCommand;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    AutoBuilder.configureHolonomic(
        drivetrainSubsystem::getCurrentPose,
        drivetrainSubsystem::setCurrentPose,
        drivetrainSubsystem::getChassisSpeeds,
        drivetrainSubsystem::drive,
        new HolonomicPathFollowerConfig(
            new PIDConstants(AutoDriveConstants.TRANSLATION_kP, AutoDriveConstants.TRANSLATION_kI,
                AutoDriveConstants.TRANSLATION_kD),
            new PIDConstants(AutoDriveConstants.THETA_kP, AutoDriveConstants.THETA_kI, AutoDriveConstants.THETA_kD),
            DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND,
            new Translation2d(DrivetrainConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0,
                DrivetrainConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0).getNorm(),
            new ReplanningConfig()),
        drivetrainSubsystem);


    // Configure control binding scheme
    if (DriverStation.getJoystickIsXbox(0)) {
      controlBindings = new XBoxControlBindings();
    } else {
      controlBindings = new JoystickControlBindings();
    }

    fieldOrientedDriveCommand = new FieldOrientedDriveCommand(
        drivetrainSubsystem,
        () -> drivetrainSubsystem.getCurrentPose().getRotation(),
        controlBindings.translationX(),
        controlBindings.translationY(),
        controlBindings.omega());

    configureDefaultCommands();
    configureButtonBindings();
    configureDashboard();
  }

  private void configureDefaultCommands() {
    // Set up the default command for the drivetrain.
    drivetrainSubsystem.setDefaultCommand(fieldOrientedDriveCommand);
  }

  private void configureDashboard() {
    SmartDashboard.putData("Drivetrain", drivetrainSubsystem);

    /**** Vision tab ****/
    final var visionTab = Shuffleboard.getTab("Vision");

    // Drivetrain / pose
    drivetrainSubsystem.addDashboardWidgets(visionTab);
  }

  private void configureButtonBindings() {
    // reset the robot pose
    controlBindings.resetPose().ifPresent(trigger -> trigger.onTrue(runOnce(drivetrainSubsystem::resetFieldPosition)));

    // POV Right to put the wheels in an X until the driver tries to drive
    controlBindings.wheelsToX()
        .ifPresent(trigger -> trigger.onTrue(run(drivetrainSubsystem::setWheelsToX, drivetrainSubsystem)
            .until(() -> controlBindings.translationX().getAsDouble() != 0
                || controlBindings.translationY().getAsDouble() != 0
                || controlBindings.omega().getAsDouble() != 0 )));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new PathPlannerAuto("Test Auto");
  }

  /**
   * Called when the alliance reported by the driverstation/FMS changes.
   * @param alliance new alliance value
   */
  public void onAllianceChanged(Alliance alliance) {
    drivetrainSubsystem.setAlliance(alliance);
  }

}
