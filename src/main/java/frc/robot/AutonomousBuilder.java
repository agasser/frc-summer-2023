package frc.robot;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.function.Consumer;
import java.util.function.Supplier;
import java.util.stream.Stream;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.AutoDriveConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AutonomousBuilder {

  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

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
            DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND,
            new Translation2d(DrivetrainConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0,
                DrivetrainConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0).getNorm(),
            new ReplanningConfig()),
        drivetrainSubsystem);

    autoChooser.setDefaultOption("None", Commands.none());
    // Add all the autos in the pathplanner directory
    try (Stream<Path> files = Files
        .list(Paths.get(Filesystem.getDeployDirectory().getAbsolutePath(), "pathplanner/autos"))) {
      files.filter(file -> !Files.isDirectory(file))
          .map(Path::getFileName)
          .map(Path::toString)
          .filter(fileName -> fileName.endsWith(".auto"))
          .sorted()
          .map(pathName -> pathName.substring(0, pathName.lastIndexOf(".")))
          .forEach(pathName -> autoChooser.addOption("PP: " + pathName, new PathPlannerAuto(pathName)));
    } catch (IOException e) {
      DriverStation.reportError("********* Failed to list PathPlanner paths. *********", false);
    }
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
