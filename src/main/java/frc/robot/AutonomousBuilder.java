package frc.robot;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.HashMap;
import java.util.function.Consumer;
import java.util.function.Supplier;
import java.util.stream.Stream;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.AutoDriveConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.DrivetrainSubsystem;

/**
 * Class for constructing autonomous routines
 */
public class AutonomousBuilder {

  private final SendableChooser<Command> autoChooser = new SendableChooser<>();
  private final SwerveAutoBuilder swerveAutoBuilder;

  public AutonomousBuilder(
      DrivetrainSubsystem drivetrainSubsystem, Supplier<Pose2d> poseSupplier, Consumer<Pose2d> poseConsumer) {
    
    var eventMap = buildEventMap();
    swerveAutoBuilder = new SwerveAutoBuilder(
        poseSupplier::get,
        poseConsumer::accept,
        DrivetrainConstants.KINEMATICS,
        new PIDConstants(AutoDriveConstants.TRANSLATION_kP, AutoDriveConstants.TRANSLATION_kI, AutoDriveConstants.TRANSLATION_kD),
        new PIDConstants(AutoDriveConstants.THETA_kP, AutoDriveConstants.THETA_kI, AutoDriveConstants.THETA_kD),
        drivetrainSubsystem::setModuleStates,
        eventMap,
        true,
        drivetrainSubsystem
    );

    autoChooser.setDefaultOption("None", Commands.none());
    // Add all the paths in the pathplanner directory that start with "Auto - "
    try (Stream<Path> files = Files.list(Paths.get(Filesystem.getDeployDirectory().getAbsolutePath(), "pathplanner"))) {
      files.filter(file -> !Files.isDirectory(file))
          .map(Path::getFileName)
          .map(Path::toString)
          .filter(fileName -> fileName.endsWith(".path"))
          .filter(fileName -> fileName.startsWith("Auto - "))
          .sorted()
          .map(pathName -> pathName.substring(0, pathName.lastIndexOf(".")))
          .forEach(pathName -> autoChooser.addOption("PP: " + pathName.substring(6), buildAutoForPathGroup(pathName)));
    } catch (IOException e) {
      System.out.println("********* Failed to list PathPlanner paths. *********");
    }

  }

  /**
   * Adds the auto chooser
   * @param dashboard dashboard
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

  private Command buildAutoForPathGroup(String pathGroupName) {
    var twoConePath = PathPlanner.loadPathGroup(pathGroupName, PathPlanner.getConstraintsFromPath(pathGroupName));
    if (twoConePath == null) {
      return Commands.print("********* Path failed to load. Not running auto: " + pathGroupName + " *********");
    }
    return swerveAutoBuilder.fullAuto(twoConePath);
  }

  private HashMap<String, Command> buildEventMap() {
    var eventMap = new HashMap<String, Command>();
    return eventMap;
  }

}
