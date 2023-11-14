package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.function.Consumer;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoDriveConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AutonomousBuilder {

  private final SendableChooser<Command> autoChooser;

  public AutonomousBuilder(DrivetrainSubsystem drivetrainSubsystem, Supplier<Pose2d> poseSupplier,
      Consumer<Pose2d> poseConsumer) {

    AutoBuilder.configureHolonomic(
        poseSupplier,
        poseConsumer,
        drivetrainSubsystem::getChassisSpeeds,
        drivetrainSubsystem::drive,
        new HolonomicPathFollowerConfig(
            new PIDConstants(AutoDriveConstants.TRANSLATION_kP, AutoDriveConstants.TRANSLATION_kI,
                AutoDriveConstants.TRANSLATION_kD),
            new PIDConstants(AutoDriveConstants.THETA_kP, AutoDriveConstants.THETA_kI, AutoDriveConstants.THETA_kD),
            DrivetrainConstants.MAX_VELOCITY.in(MetersPerSecond),
            new Translation2d(DrivetrainConstants.WHEELBASE.in(Meters) / 2.0,
                DrivetrainConstants.TRACKWIDTH.in(Meters) / 2.0).getNorm(),
            new ReplanningConfig()),
        drivetrainSubsystem);

    autoChooser = AutoBuilder.buildAutoChooser();
  }

  /**
   * Adds the auto chooser
   * 
   * @param dashboard
   *          dashboard
   */
  public void addDashboardWidgets(ShuffleboardTab dashboard) {
    dashboard.add("Autonomous", autoChooser).withSize(2, 1).withPosition(4, 3);
  }

  /**
   * Gets the currently selected auto command
   * @return auto command
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
