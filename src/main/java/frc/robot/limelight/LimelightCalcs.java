package frc.robot.limelight;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.limelight.model.LimelightDetectorTarget;
import frc.robot.limelight.model.LimelightRetroTarget;

/**
 * Class to perform calculations from a limelight target
 */
public class LimelightCalcs {

  private final Transform3d cameraToRobot;
  private final Transform2d robotToCamera2d;
  private final double targetHeight;

  /**
   * Constructor
   * @param cameraToRobot transform from the camera to the robot
   * @param targetHeight height of the target
   */
  public LimelightCalcs(Transform3d cameraToRobot, double targetHeight) {
    this.cameraToRobot = cameraToRobot;
    this.robotToCamera2d = new Transform2d(
        cameraToRobot.getTranslation().toTranslation2d(),
        cameraToRobot.getRotation().toRotation2d()).inverse();
    this.targetHeight = targetHeight;
  }

  /**
   * Gets target info, relative to the robot.
   * @param targetXDegrees X coordinate of the target in degrees
   * @param targetYDegrees Y coordinate of the target in degrees
   * @return robot relative target
   */
  public VisionTargetInfo getRobotRelativeTargetInfo(double targetXDegrees, double targetYDegrees) {
    var rolledAngles = new Translation2d(targetXDegrees, targetYDegrees)
        .rotateBy(new Rotation2d(-cameraToRobot.getRotation().getX()));
    var translation = getTargetTranslation(rolledAngles.getX(), rolledAngles.getY());
    var distance = translation.getDistance(new Translation2d());
    var angle = new Rotation2d(translation.getX(), translation.getY());
    return new VisionTargetInfo(translation, distance, angle);
  }

  /**
   * Gets target info, relative to the robot.
   * @param retroTarget limelight target data
   * @return robot relative target
   */
  public VisionTargetInfo getRobotRelativeTargetInfo(LimelightRetroTarget retroTarget) {
    return getRobotRelativeTargetInfo(retroTarget.targetXDegrees, retroTarget.targetYDegrees);
  }

  /**
   * Gets target info, relative to the robot.
   * @param detectorTarget limelight target data
   * @return robot relative target
   */
  public VisionTargetInfo getRobotRelativeTargetInfo(LimelightDetectorTarget detectorTarget) {
    return getRobotRelativeTargetInfo(detectorTarget.targetXDegrees, detectorTarget.targetYDegrees);
  }

  /**
   * Get distance from camera to target, along the floor
   * @param targetYDegrees Y coordinate of the target in degrees
   * @return distance from the camera to the target
   */
  protected double getCameraToTargetDistance(double targetYDegrees) {
    var cameraPitch = -cameraToRobot.getRotation().getY();
    var cameraHeight = -cameraToRobot.getZ();
    return (targetHeight - cameraHeight)
        / Math.tan(cameraPitch + Units.degreesToRadians(targetYDegrees));
  }

  /**
   * Gets the robot relative translation of the target
   * @param targetXDegrees X coordinate of the target in degrees
   * @param targetYDegrees Y coordinate of the target in degrees
   * @return robot relative translaction
   */
  protected Translation2d getTargetTranslation(double targetXDegrees, double targetYDegrees) {
    var targetOnCameraCoordinates = new Translation2d(
        getCameraToTargetDistance(targetYDegrees),
        Rotation2d.fromDegrees(-targetXDegrees));

    return targetOnCameraCoordinates.plus(robotToCamera2d.getTranslation().rotateBy(robotToCamera2d.getRotation()));
  }

}