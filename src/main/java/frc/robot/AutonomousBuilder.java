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
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.commands.autonomous.DriveToPoseCommand;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AutonomousBuilder {
  private final DrivetrainSubsystem drivetrainSubsystem;
  private final Supplier<Pose2d> poseSupplier;

  private final SendableChooser<Command> autoChooser = new SendableChooser<>();
  private final SwerveAutoBuilder swerveAutoBuilder;

  public AutonomousBuilder(
      DrivetrainSubsystem drivetrainSubsystem, Supplier<Pose2d> poseSupplier, Consumer<Pose2d> poseConsumer) {
    
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.poseSupplier = poseSupplier;

    var eventMap = buildEventMap();
    swerveAutoBuilder = new SwerveAutoBuilder(
        poseSupplier::get,
        poseConsumer::accept,
        DrivetrainConstants.KINEMATICS,
        new PIDConstants(AutoConstants.X_kP, AutoConstants.X_kI, AutoConstants.X_kD),
        new PIDConstants(AutoConstants.PATH_THETA_kP, AutoConstants.PATH_THETA_kI, AutoConstants.PATH_THETA_kD),
        drivetrainSubsystem::setModuleStates,
        eventMap,
        true,
        drivetrainSubsystem
    );

    autoChooser.setDefaultOption("None", Commands.none());
    // Add all the paths in the pathplanner directory
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

  public Command driveToPose(Pose2d pose, TrapezoidProfile.Constraints xyConstraints,
      TrapezoidProfile.Constraints omegaConstraints, boolean useAllianceColor) {
    
    return new DriveToPoseCommand(drivetrainSubsystem, poseSupplier::get, pose, xyConstraints,
        omegaConstraints, useAllianceColor);
  }

  /**
   * Wraps the given command so that it will end if the robot's X coordinate becomes greater than the given value.
   * @param command command to wrap
   * @param x x value
   * @return wrapped command
   */
  public Command keepingXBelow(Command command, double x) {
    return command.until(() -> poseSupplier.get().getX() > x);
  }

  /**
   * Wraps the given command so that it will end if the robot's X coordinate becomes less than the given value.
   * @param command command to wrap
   * @param x x value
   * @return wrapped command
   */
  public Command keepingXAbove(Command command, double x) {
    return command.until(() -> poseSupplier.get().getX() < x);
  }

}
