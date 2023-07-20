package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.autonomous.DriveToPoseCommand;
import frc.robot.subsystems.DrivetrainSubsystem;

/**
 * Class that can be used to easily create new instances of commands this robot implements.
 */
public class CommandBuilder {

  private final DrivetrainSubsystem drivetrainSubsystem;
  private final Supplier<Pose2d> poseSupplier;

  public CommandBuilder(DrivetrainSubsystem drivetrainSubsystem, Supplier<Pose2d> poseSupplier) {
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.poseSupplier = poseSupplier;
  }
  
  public Command driveToPose(Pose2d pose, TrapezoidProfile.Constraints xyConstraints,
      TrapezoidProfile.Constraints omegaConstraints, boolean useAllianceColor) {
    
    return new DriveToPoseCommand(drivetrainSubsystem, poseSupplier::get, pose, xyConstraints,
        omegaConstraints, useAllianceColor);
  }

  /**
   * Wraps the given command so that it will end if the robot's X coordinate becomes greater than the given value.
   * This is useful in autonomous to prevent the robot from driving onto the opponent's side of the field.
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
